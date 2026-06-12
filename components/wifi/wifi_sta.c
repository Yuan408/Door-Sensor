#include "wifi_sta.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"

/* menuconfig 中的 WiFi 配置 */
#define WIFI_SSID       CONFIG_WIFI_SSID
#define WIFI_PASSWORD   CONFIG_WIFI_PASSWORD
#define WIFI_CONN_MAX_RETRY  5

static const char *TAG = "wifi_sta";

/* FreeRTOS 事件组：BIT0 表示已连接，BIT1 表示连接失败 */
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

static int s_retry_num = 0;

/* WiFi 和 IP 事件回调 */
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_CONN_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "Connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* 初始化 ESP32 WiFi Station 模式并连接到指定 AP
 *
 * 连接流程：
 *   1. 创建事件组（用于同步等待连接结果）
 *   2. 初始化 NVS + TCP/IP 协议栈 + 默认事件循环
 *   3. 注册 WiFi 事件回调（STA_START → 触发连接，STA_DISCONNECTED → 重试）
 *   4. 配置 SSID/密码并启动 WiFi
 *   5. 事件组阻塞等待（最多 30 秒）直到连接成功或失败
 *
 * 事件回调驱动状态机：
 *   WIFI_EVENT_STA_START    → esp_wifi_connect()
 *   WIFI_EVENT_STA_DISCONNECTED → 重试（最多 5 次）或设置 FAIL 标志
 *   IP_EVENT_STA_GOT_IP     → 设置 CONNECTED 标志，连接成功
 */
esp_err_t wifi_init_sta(void)
{
    /* 创建事件组：用于同步等待连接结果 */
    s_wifi_event_group = xEventGroupCreate();

    /* 初始化 NVS（WiFi 库需要 NVS 存储配置） */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 初始化 TCP/IP 协议栈和默认事件循环 */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();  /* 创建默认 WiFi STA 接口 */

    /* WiFi 初始化（使用默认配置） */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* 注册 WiFi 和 IP 事件回调 */
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    /* 配置 WiFi 为 STA 模式，指定 SSID 和密码（来自 menuconfig） */
    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,  /* 最低安全要求 */
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());  /* 启动 WiFi，触发 WIFI_EVENT_STA_START */

    ESP_LOGI(TAG, "WiFi STA init finished, connecting to SSID: %s", WIFI_SSID);

    /* 阻塞等待连接完成（最多 30 秒） */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,    /* 满足任一条件即返回 */
                                           pdFALSE,    /* 不清除事件位 */
                                           pdMS_TO_TICKS(30000));  /* 30 秒超时 */

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP SSID: %s", WIFI_SSID);
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to SSID: %s", WIFI_SSID);
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");  /* 超时：既没成功也没失败 */
        return ESP_FAIL;
    }
}