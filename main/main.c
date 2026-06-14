#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include "sht30.h"
#include "ssd1306.h"
#include "wifi_sta.h"
#include "ota_manager.h"
#include "watchdog.h"
#include "can_bus.h"
#include "nvs_flash.h"  /* NVS Flash 存储初始化（掉电不丢失） */
#include "nvs.h"        /* NVS 读写 API */

/* I2C 引脚配置（I2C0：SHT30_A 本地 + OLED 共用） */
#define I2C_MASTER_SCL_IO        22   /* I2C 时钟线 GPIO */
#define I2C_MASTER_SDA_IO        21   /* I2C 数据线 GPIO */
#define I2C_MASTER_FREQ_HZ       100000 /* I2C 标准模式 100 kHz */
#define I2C_MASTER_PORT          I2C_NUM_0 /* ESP32 I2C 控制器 0 */

/* I2C 引脚配置（I2C1：SHT30_B 远端专用） */
#define I2C_REMOTE_SCL_IO        16
#define I2C_REMOTE_SDA_IO        17
#define I2C_REMOTE_PORT          I2C_NUM_1

/* 远端 SHT30 原始读数，供 CAN 轮询任务读取 */
float g_remote_raw_temp = 0.0f;
float g_remote_raw_humi = 0.0f;

/* 滑动平均窗口大小（5 点：当前值 + 前 4 个历史值取平均） */
#define MA_FILTER_WINDOW         5

/* MQTT 配置（使用 EMQX 公共测试 Broker，生产环境请替换为私有服务器） */
#define MQTT_BROKER_URI  "mqtt://broker.emqx.io:1883" /* MQTT Broker 地址与端口 */
#define MQTT_PUB_TOPIC   "/yby/device001/sensor"      /* 发布传感器数据的主题 */
#define MQTT_SUB_TOPIC   "/cmd"                        /* 订阅远程命令的主题 */

static const char *TAG = "main";

/* MQTT 客户端句柄 */
static esp_mqtt_client_handle_t g_mqtt_client = NULL;

/* 环形缓冲滑动平均结构体
 * buffer[] —— 定长环形数组，存储最近 N 个采样值
 * index    —— 下一次写入位置（环形：到达 MA_FILTER_WINDOW 后回绕到 0）
 * count    —— 当前已存储的有效数据个数（未满窗口时 < MA_FILTER_WINDOW）
 * sum      —— 当前缓冲区所有有效值的累加和，避免每次重新遍历求和 */
typedef struct {
    float buffer[MA_FILTER_WINDOW];
    int   index;
    int   count;
    float sum;
} moving_average_t;

/* 初始化滑动平均结构体：清零缓冲区、复位索引和累加器 */
static void ma_init(moving_average_t *ma)
{
    memset(ma->buffer, 0, sizeof(ma->buffer));
    ma->index = 0;
    ma->count = 0;
    ma->sum   = 0.0f;
}

/* 插入新值并返回当前滑动平均值
 * 使用局部变量（idx/cnt/total）减少对结构体成员的频繁访问，
 * 全部逻辑操作完成后统一写回，代码更清晰易读 */
static float ma_update(moving_average_t *ma, float new_val)
{
    /* 步骤1：将结构体成员拷贝到局部变量 */
    int   idx   = ma->index;
    int   cnt   = ma->count;
    float total = ma->sum;

    if (cnt < MA_FILTER_WINDOW) {
        /* 步骤2a：窗口未满——直接将新值追加到缓冲区尾部 */
        ma->buffer[idx] = new_val;
        total += new_val;   /* 累加新值到总和 */
        cnt++;              /* 有效数据计数 +1 */
        idx++;              /* 写指针后移 */
    } else {
        /* 步骤2b：窗口已满——覆盖最旧的数据，实现滑动效果 */
        if (idx >= MA_FILTER_WINDOW) {
            idx = 0;        /* 环形缓冲区回绕到开头 */
        }
        total -= ma->buffer[idx];   /* 从总和中减去即将被覆盖的旧值 */
        ma->buffer[idx] = new_val;  /* 用新值覆盖旧值 */
        total += new_val;           /* 把新值加到总和中 */
        idx++;                      /* 写指针后移 */
    }

    /* 步骤3：将局部变量写回结构体 */
    ma->index = idx;
    ma->count = cnt;
    ma->sum   = total;

    /* 步骤4：返回当前窗口内所有有效数据的算术平均值 */
    return total / (float)cnt;
}

/* ===================================================================
 *  MQTT 事件回调
 * =================================================================== */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                                int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        /* 成功连接到 Broker 后，订阅命令主题以接收远程指令 */
        ESP_LOGI(TAG, "MQTT connected");
        esp_mqtt_client_subscribe(g_mqtt_client, MQTT_SUB_TOPIC, 0);
        ESP_LOGI(TAG, "Subscribed to topic: %s", MQTT_SUB_TOPIC);
        break;

    case MQTT_EVENT_DISCONNECTED:
        /* 断连后 ESP-MQTT 库会自动重连，此处仅记录日志 */
        ESP_LOGI(TAG, "MQTT disconnected, reconnecting...");
        break;

    case MQTT_EVENT_DATA: {
        /* 收到订阅主题的消息 */
        /* 提取 payload 字符串（MQTT 消息可能不包含 null-terminator） */
        char payload[256] = {0};
        int copy_len = event->data_len < sizeof(payload) - 1
                       ? event->data_len : sizeof(payload) - 1;
        memcpy(payload, event->data, copy_len);
        ESP_LOGI(TAG, "MQTT message received: topic=%.*s, payload=%s",
                 event->topic_len, event->topic, payload);

        /* 解析 JSON，检查是否为 OTA 升级命令
         * 期望格式：{"cmd":"ota", "url":"http://.../firmware.bin"} */
        cJSON *root = cJSON_Parse(payload);
        if (root == NULL) {
            ESP_LOGW(TAG, "Failed to parse MQTT message JSON");
            break;
        }

        cJSON *cmd = cJSON_GetObjectItem(root, "cmd");
        if (cmd && cJSON_IsString(cmd) && strcmp(cmd->valuestring, "ota") == 0) {
            /* 收到 OTA 命令，提取固件下载地址 */
            cJSON *url = cJSON_GetObjectItem(root, "url");
            if (url && cJSON_IsString(url)) {
                ESP_LOGI(TAG, "OTA command received, url=%s", url->valuestring);
                ota_trigger(url->valuestring);  /* 触发 OTA 升级流程 */
            } else {
                ESP_LOGW(TAG, "OTA command missing 'url' field");
            }
        } else {
            ESP_LOGI(TAG, "Unknown or missing 'cmd' in MQTT message");
        }

        cJSON_Delete(root);  /* 释放 cJSON 对象 */
        break;
    }

    case MQTT_EVENT_ERROR:
        /* MQTT 通信错误 */
        ESP_LOGE(TAG, "MQTT error");
        break;

    default:
        break;
    }
}

/* ===================================================================
 *  MQTT 发布传感器数据
 *  JSON 格式：{"temperature": 25.5, "humidity": 60.2, "timestamp": 1717000000}
 *  使用 snprintf 直接构造 JSON 字符串，精确控制小数位（1 位）。
 * =================================================================== */
static void mqtt_publish_sensor_data(float temperature, float humidity)
{
    if (g_mqtt_client == NULL) {
        return;
    }

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return;

    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", temperature);
    cJSON_AddNumberToObject(root, "temperature", atof(buf));
    snprintf(buf, sizeof(buf), "%.1f", humidity);
    cJSON_AddNumberToObject(root, "humidity", atof(buf));
    cJSON_AddNumberToObject(root, "timestamp", (double)time(NULL));
    snprintf(buf, sizeof(buf), "%.1f", g_can_temp);
    cJSON_AddNumberToObject(root, "remote_temperature", atof(buf));
    snprintf(buf, sizeof(buf), "%.1f", g_can_humi);
    cJSON_AddNumberToObject(root, "remote_humidity", atof(buf));

    char *json_str = cJSON_PrintUnformatted(root);
    if (json_str != NULL) {
        int msg_id = esp_mqtt_client_publish(g_mqtt_client, MQTT_PUB_TOPIC,
                                              json_str, 0, 1, 0);
        if (msg_id < 0) {
            ESP_LOGW(TAG, "MQTT publish failed");
        } else {
            ESP_LOGI(TAG, "MQTT published: %s", json_str);
        }
        cJSON_free(json_str);
    }
    cJSON_Delete(root);

    /* 报告网络任务存活（MQTT 活跃即网络正常） */
    watchdog_report_alive(WATCHDOG_TASK_NETWORK);
}

/* ===================================================================
 *  MQTT 客户端启动
 *  连接 EMQX 公共 Broker（mqtt://broker.emqx.io:1883），
 *  注册事件回调后启动客户端。
 * =================================================================== */
static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };

    g_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (g_mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return;
    }

    esp_mqtt_client_register_event(g_mqtt_client, ESP_EVENT_ANY_ID,
                                    mqtt_event_handler, NULL);

    esp_err_t ret = esp_mqtt_client_start(g_mqtt_client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "MQTT client started, broker: %s", MQTT_BROKER_URI);
}

/* ===================================================================
 *  NVS 辅助函数 —— 使用 NVS（Non-Volatile Storage）存储开机次数
 *  命名空间: "storage"    键名: "boot_count"
 * =================================================================== */

/**
 * @brief 保存开机次数到 NVS
 * @param count 要保存的开机计数值
 * @return ESP_OK 成功，其他值表示失败
 */
static esp_err_t nvs_save_boot_count(int32_t count)
{
    nvs_handle_t handle;

    /* 以读写模式打开 NVS 命名空间 */
    esp_err_t ret = nvs_open("storage", NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;

    /* 写入 int32_t 类型的键值对 */
    ret = nvs_set_i32(handle, "boot_count", count);
    if (ret != ESP_OK) {
        nvs_close(handle);  /* 失败时必须关闭句柄 */
        return ret;
    }

    /* 提交更改，确保数据写入 Flash */
    ret = nvs_commit(handle);
    nvs_close(handle);      /* 关闭句柄释放资源 */
    return ret;
}

/**
 * @brief 从 NVS 读取开机次数
 * @param count [out] 指向存储读取结果的变量指针
 * @return ESP_OK 成功，其他值表示失败（键不存在时 *count 保持 0）
 */
static esp_err_t nvs_load_boot_count(int32_t *count)
{
    nvs_handle_t handle;

    /* 以只读模式打开 NVS 命名空间 */
    esp_err_t ret = nvs_open("storage", NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        *count = 0;         /* 命名空间不存在，开机次数视为 0 */
        return ret;
    }

    /* 读取键值，若键不存在则 *count 保持为 0 */
    ret = nvs_get_i32(handle, "boot_count", count);
    if (ret != ESP_OK) {
        *count = 0;         /* 键不存在或读取失败，重置为 0 */
    }
    nvs_close(handle);      /* 关闭句柄释放资源 */
    return ret;
}

/* ===================================================================
 *  主函数
 * =================================================================== */
void app_main(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "App version: 1 stable");

    /* ==================== NVS 初始化 ==================== */
    /* NVS（Non-Volatile Storage）用于保存配置和运行状态，掉电不丢失 */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* NVS 分区空间不足或版本更新，需要擦除后重新初始化 */
        ESP_LOGW(TAG, "NVS 分区需要格式化，正在擦除...");
        ESP_ERROR_CHECK(nvs_flash_erase());   /* 擦除整个 NVS 分区 */
        ret = nvs_flash_init();               /* 重新初始化 */
    }
    ESP_ERROR_CHECK(ret);                     /* 检查最终结果，失败则复位 */
    ESP_LOGI(TAG, "NVS 初始化完成");

    /* ==================== WiFi 初始化 ==================== */
    ESP_LOGI(TAG, "Initializing WiFi...");
    wifi_init_sta();

    /* ==================== MQTT 初始化 ==================== */
    mqtt_app_start();

    /* ==================== I2C 总线初始化 ==================== */
    i2c_master_bus_config_t i2c_bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port   = I2C_MASTER_PORT,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle = NULL;
    ret = i2c_new_master_bus(&i2c_bus_cfg, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "I2C bus initialized (SDA=%d, SCL=%d, %d Hz)",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);

    /* ==================== 看门狗初始化 ==================== */
    watchdog_init();

    /* ==================== SHT30 初始化 ==================== */
    i2c_master_dev_handle_t sht30_dev = NULL;
    ret = sht30_init(bus_handle, &sht30_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SHT30 init failed");
        return;
    }

    /* ==================== SSD1306 初始化 ==================== */
    i2c_master_dev_handle_t oled_dev = NULL;
    ret = ssd1306_init(bus_handle, &oled_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SSD1306 init failed");
        return;
    }

    /* ==================== 远端 I2C 总线初始化（I2C1：GPIO16/17） ==================== */
    i2c_master_bus_config_t i2c_remote_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port   = I2C_REMOTE_PORT,
        .scl_io_num = I2C_REMOTE_SCL_IO,
        .sda_io_num = I2C_REMOTE_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle_remote = NULL;
    ret = i2c_new_master_bus(&i2c_remote_cfg, &bus_handle_remote);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C remote bus init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "I2C remote bus initialized (SDA=%d, SCL=%d)",
             I2C_REMOTE_SDA_IO, I2C_REMOTE_SCL_IO);

    /* ==================== 远端 SHT30_B 初始化 ==================== */
    i2c_master_dev_handle_t sht30_dev_remote = NULL;
    ret = sht30_init(bus_handle_remote, &sht30_dev_remote);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SHT30 remote init failed");
        return;
    }

    /* ==================== CAN 总线初始化 ==================== */
#ifdef CONFIG_ENABLE_CAN
    ret = can_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CAN init failed, continuing without CAN");
    } else {
        can_start_poll();    /* background polling of remote sensor */
        ESP_LOGI(TAG, "CAN bus ready");
    }
#endif

    /* ==================== 初始化滑动平均 ==================== */
    moving_average_t ma_temp;
    moving_average_t ma_humi;
    ma_init(&ma_temp);
    ma_init(&ma_humi);

    /* ==================== 开机计数（NVS 掉电存储演示） ==================== */
    int32_t boot_count = 0;
    /* 从 NVS 读取上次保存的开机次数，若读取失败则从 0 开始计数 */
    if (nvs_load_boot_count(&boot_count) != ESP_OK) {
        boot_count = 0;
        printf("首次启动，开机计数初始化为 0\n");
    }
    boot_count++;                            /* 本次开机 +1 */
    nvs_save_boot_count(boot_count);         /* 保存到 NVS，掉电后依然保留 */
    printf("【开机计数】当前是第 %ld 次启动\n", (long)boot_count);

    /* ==================== 主循环 ==================== */
    while (1) {
        /* 第1步：读取 SHT30_A 本地原始数据 */
        float raw_temp = 0.0f;
        float raw_humi = 0.0f;

        ret = sht30_read_temp_humidity(sht30_dev, &raw_temp, &raw_humi);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "SHT30 local read failed, retrying...");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        /* 第2步：5 次滑动平均滤波（去除传感器噪声） */
        float avg_temp = ma_update(&ma_temp, raw_temp);
        float avg_humi = ma_update(&ma_humi, raw_humi);

        /* 第3步：读取 SHT30_B 远端原始数据（供 CAN 轮询任务发送） */
        float r_temp = 0.0f, r_humi = 0.0f;
        if (sht30_read_temp_humidity(sht30_dev_remote, &r_temp, &r_humi) == ESP_OK) {
            g_remote_raw_temp = r_temp;
            g_remote_raw_humi = r_humi;
        }

        /* 第4步：报告传感器任务存活（喂狗心跳） */
        watchdog_report_alive(WATCHDOG_TASK_SENSOR);

        /* 第5步：MQTT 发布滤波后的温湿度数据 */
        mqtt_publish_sensor_data(avg_temp, avg_humi);

        /* 第6步：串口调试输出 */
        printf("SHT30: Local=%.2fC %.2f%%, Remote=%.2fC %.2f%%\n",
               avg_temp, avg_humi, g_remote_raw_temp, g_remote_raw_humi);

        /* 第7步：OLED 显示 —— L 行本地，R 行 CAN 远端（来自回环帧） */
        char line1[17] = {0};
        char line2[17] = {0};
        snprintf(line1, sizeof(line1), "U:%.1fC %.1f%%", avg_temp, avg_humi);
#ifdef CONFIG_ENABLE_CAN
        if (g_can_temp > -99.0f || g_can_humi > -99.0f) {
            snprintf(line2, sizeof(line2), "D:%.1fC %.1f%%", g_can_temp, g_can_humi);
        } else {
            snprintf(line2, sizeof(line2), "D: --");
        }
#else
        snprintf(line2, sizeof(line2), "H:%.1f %%", avg_humi);
#endif
        ssd1306_display_string(line1, line2);

        /* 第8步：报告显示任务存活（喂狗心跳） */
        watchdog_report_alive(WATCHDOG_TASK_DISPLAY);

        /* 延时 2 秒后进入下一次循环 */
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
