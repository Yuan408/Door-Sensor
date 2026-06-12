#include "can_bus.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

static const char *TAG = "can_bus";

/* ===================================================================
 *  MCP2515 SPI commands & register map
 * =================================================================== */
#define MCP_CMD_RESET       0xC0
#define MCP_CMD_READ        0x03
#define MCP_CMD_WRITE       0x02
#define MCP_CMD_RTS(n)      (0x80 | (1 << (n)))   /* n = 0,1,2 */
#define MCP_CMD_READ_STATUS 0xA0
#define MCP_CMD_RX_STATUS   0xB0
#define MCP_CMD_BIT_MODIFY  0x05

/* -- register addresses -- */
#define MCP_REG_CANSTAT     0x0E
#define MCP_REG_CANCTRL     0x0F
#define MCP_REG_CNF3        0x28
#define MCP_REG_CNF2        0x29
#define MCP_REG_CNF1        0x2A
#define MCP_REG_CANINTE     0x2B
#define MCP_REG_CANINTF     0x2C
#define MCP_REG_EFLG        0x2D
#define MCP_REG_TEC         0x1C
#define MCP_REG_REC         0x1D

#define MCP_REG_TXB0CTRL    0x30
#define MCP_REG_TXB0SIDH    0x31
#define MCP_REG_TXB0SIDL    0x32
#define MCP_REG_TXB0EID8    0x33
#define MCP_REG_TXB0EID0    0x34
#define MCP_REG_TXB0DLC     0x35
#define MCP_REG_TXB0D0      0x36

#define MCP_REG_RXB0CTRL    0x60
#define MCP_REG_RXB0SIDH    0x61
#define MCP_REG_RXB0SIDL    0x62
#define MCP_REG_RXB0DLC     0x65
#define MCP_REG_RXB0D0      0x66

#define MCP_REG_RXB1CTRL    0x70
#define MCP_REG_RXB1SIDH    0x71
#define MCP_REG_RXB1SIDL    0x72
#define MCP_REG_RXB1DLC     0x75
#define MCP_REG_RXB1D0      0x76

/* -- mode bits in CANCTRL -- */
#define MCP_REQOP_MASK      0xE0
#define MCP_REQOP_NORMAL    0x00
#define MCP_REQOP_SLEEP     0x20
#define MCP_REQOP_LOOPBACK  0x40
#define MCP_REQOP_LISTEN    0x60
#define MCP_REQOP_CONFIG    0x80

/* -- CANINTF bits -- */
#define MCP_INT_RX0IF       (1 << 0)
#define MCP_INT_RX1IF       (1 << 1)
#define MCP_INT_TX0IF       (1 << 2)
#define MCP_INT_TX1IF       (1 << 3)
#define MCP_INT_TX2IF       (1 << 4)
#define MCP_INT_ERRIF       (1 << 5)
#define MCP_INT_WAKIF       (1 << 6)
#define MCP_INT_MERRF       (1 << 7)

/* ===================================================================
 *  Global state
 * =================================================================== */
float g_can_temp = 0.0f;
float g_can_humi = 0.0f;

static spi_device_handle_t s_spi_dev = NULL;
static SemaphoreHandle_t   s_spi_mutex = NULL;
static TaskHandle_t        s_poll_task = NULL;

/* reuseable transaction descriptor, only used from locked contexts */
static spi_transaction_t s_trans = {0};

/* ===================================================================
 *  Low-level SPI helpers (caller must hold s_spi_mutex)
 * =================================================================== */
static esp_err_t mcp_spi_transfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    s_trans.length    = len * 8;
    s_trans.rxlength  = len * 8;
    s_trans.tx_buffer = tx;
    s_trans.rx_buffer = rx;
    return spi_device_transmit(s_spi_dev, &s_trans);
}

static void mcp_write_reg(uint8_t addr, uint8_t val)
{
    uint8_t tx[3] = { MCP_CMD_WRITE, addr, val };
    mcp_spi_transfer(tx, NULL, sizeof(tx));
}

static uint8_t mcp_read_reg(uint8_t addr)
{
    uint8_t tx[3] = { MCP_CMD_READ, addr, 0x00 };
    uint8_t rx[3] = {0};
    mcp_spi_transfer(tx, rx, sizeof(tx));
    return rx[2];
}

static void mcp_bit_modify(uint8_t addr, uint8_t mask, uint8_t val)
{
    uint8_t tx[4] = { MCP_CMD_BIT_MODIFY, addr, mask, val };
    mcp_spi_transfer(tx, NULL, sizeof(tx));
}

static uint8_t mcp_read_status(void)
{
    uint8_t tx[2] = { MCP_CMD_READ_STATUS, 0x00 };
    uint8_t rx[2] = {0};
    mcp_spi_transfer(tx, rx, sizeof(tx));
    return rx[1];
}

static void mcp_reset(void)
{
    uint8_t tx = MCP_CMD_RESET;
    mcp_spi_transfer(&tx, NULL, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

/* ===================================================================
 *  MCP2515 init sequence
 * =================================================================== */
static bool mcp_set_mode(uint8_t mode)
{
    mcp_bit_modify(MCP_REG_CANCTRL, MCP_REQOP_MASK, mode);

    /* wait for mode switch (max 10 ms) */
    for (int i = 0; i < 10; i++) {
        uint8_t stat = mcp_read_reg(MCP_REG_CANSTAT) & MCP_REQOP_MASK;
        if (stat == mode) return true;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

static void mcp_set_baud_rate(void)
{
    /* Bit-timing table for 8 MHz MCP2515 crystal.
       SyncSeg is always 1 TQ.
       BRP, PropSeg, PS1, PS2 values stored as (actual - 1). */
#if defined(CONFIG_CAN_SPEED_125K)
    /* 125kbps with 8MHz crystal: BRP=4 TQ=1us NBT=16, SJW=1, 采样点 75% */
    uint8_t cnf1 = 0x03;
    uint8_t cnf2 = 0x9E;
    uint8_t cnf3 = 0x03;
#elif defined(CONFIG_CAN_SPEED_250K)
    /* 250 kbps : BRP=1 -> TQ=250 ns, NBT=16: Prop=7 PS1=4 PS2=4 SJW=1 */
    uint8_t cnf1 = 0x01;
    uint8_t cnf2 = 0x9E;
    uint8_t cnf3 = 0x03;
#elif defined(CONFIG_CAN_SPEED_500K)
    /* 500 kbps : BRP=0 -> TQ=125 ns, NBT=16: Prop=7 PS1=4 PS2=4 SJW=1 */
    uint8_t cnf1 = 0x00;
    uint8_t cnf2 = 0x9E;
    uint8_t cnf3 = 0x03;
#else
    /* default 125 kbps */
    uint8_t cnf1 = 0x03;
    uint8_t cnf2 = 0x9E;
    uint8_t cnf3 = 0x03;
#endif

    mcp_write_reg(MCP_REG_CNF1, cnf1);
    mcp_write_reg(MCP_REG_CNF2, cnf2);
    mcp_write_reg(MCP_REG_CNF3, cnf3);
}

static esp_err_t mcp2515_init(void)
{
    xSemaphoreTake(s_spi_mutex, portMAX_DELAY);

    mcp_reset();
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Enter config mode */
    if (!mcp_set_mode(MCP_REQOP_CONFIG)) {
        xSemaphoreGive(s_spi_mutex);
        ESP_LOGE(TAG, "Failed to enter config mode");
        return ESP_FAIL;
    }

    mcp_set_baud_rate();

    /* Accept all messages on both RX buffers (no filtering) */
    mcp_write_reg(MCP_REG_RXB0CTRL, 0x60);  /* receive any, no RTR flag */
    mcp_write_reg(MCP_REG_RXB1CTRL, 0x60);

    /* Clear interrupt flags, disable all interrupts (polled mode) */
    mcp_bit_modify(MCP_REG_CANINTF, 0xFF, 0x00);
    mcp_write_reg(MCP_REG_CANINTE, 0x00);

    /* Enter selected mode */
#ifdef CONFIG_CAN_MODE_LOOPBACK
    uint8_t op_mode = MCP_REQOP_LOOPBACK;
    const char *mode_str = "loopback";
#elif defined(CONFIG_CAN_MODE_LISTEN)
    uint8_t op_mode = MCP_REQOP_LISTEN;
    const char *mode_str = "listen-only";
#else
    uint8_t op_mode = MCP_REQOP_NORMAL;
    const char *mode_str = "normal";
#endif

    if (!mcp_set_mode(op_mode)) {
        xSemaphoreGive(s_spi_mutex);
        ESP_LOGE(TAG, "Failed to enter %s mode", mode_str);
        return ESP_FAIL;
    }

    xSemaphoreGive(s_spi_mutex);
    ESP_LOGI(TAG, "MCP2515 initialized in %s mode", mode_str);
    return ESP_OK;
}

/* ===================================================================
 *  Public API
 * =================================================================== */
/* 初始化 CAN 总线（SPI + MCP2515）
 * 引脚分配：MOSI=GPIO23, MISO=GPIO19, SCLK=GPIO18, CS=GPIO5
 * 初始化流程：
 *   1. 初始化 ESP32 SPI2 主机（10 MHz, SPI Mode 0）
 *   2. 添加 MCP2515 从设备
 *   3. 创建互斥锁保护 SPI 总线
 *   4. 初始化 MCP2515（复位 → 配置模式 → 设置波特率 → 运行模式） */
esp_err_t can_init(void)
{
    /* SPI 总线配置：标准四线 SPI，禁用 WP/HD 引脚 */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = 23,   /* 主机输出 → MCP2515 SI */
        .miso_io_num     = 19,   /* 主机输入 ← MCP2515 SO */
        .sclk_io_num     = 18,   /* 时钟信号 → MCP2515 SCK */
        .quadwp_io_num   = -1,   /* 不使用 */
        .quadhd_io_num   = -1,   /* 不使用 */
        .max_transfer_sz = 64,   /* 最大传输字节数 */
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_DISABLED);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 添加 MCP2515 从设备 */
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 1 * 1000 * 1000,    /* 杜邦线连接时降低 SPI 时钟，避免读帧位错误 */
        .mode           = 0,                   /* CPOL=0, CPHA=0（SPI Mode 0） */
        .spics_io_num   = 5,                   /* 片选信号 GPIO5 */
        .queue_size     = 7,                   /* 事务队列深度 */
        .address_bits   = 0,
        .dummy_bits     = 0,
    };

    ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &s_spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        spi_bus_free(SPI2_HOST);
        return ret;
    }

    /* 创建互斥锁：所有 SPI 操作必须持有锁，避免并发冲突 */
    s_spi_mutex = xSemaphoreCreateMutex();
    if (s_spi_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create SPI mutex");
        return ESP_ERR_NO_MEM;
    }

    /* MCP2515 芯片初始化（复位 → 配置 → 波特率 → 运行模式） */
    ret = mcp2515_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MCP2515 init failed");
        return ret;
    }

    ESP_LOGI(TAG, "CAN bus ready");
    return ESP_OK;
}

esp_err_t can_send(uint32_t id, uint8_t *data, uint8_t len)
{
    if (len > 8 || s_spi_dev == NULL) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_spi_mutex, portMAX_DELAY);

    /* Abort any pending TXB0 transmission, clear TX0IF */
    mcp_bit_modify(MCP_REG_TXB0CTRL, 0x08, 0x00);  /* clear TXREQ */
    mcp_bit_modify(MCP_REG_CANINTF, MCP_INT_TX0IF, 0x00);

    /* Load ID (standard 11-bit) */
    mcp_write_reg(MCP_REG_TXB0SIDH, (uint8_t)(id >> 3));
    mcp_write_reg(MCP_REG_TXB0SIDL, (uint8_t)(id << 5));
    mcp_write_reg(MCP_REG_TXB0EID8, 0x00);
    mcp_write_reg(MCP_REG_TXB0EID0, 0x00);

    /* Load DLC and data */
    mcp_write_reg(MCP_REG_TXB0DLC, len);
    for (int i = 0; i < len; i++) {
        mcp_write_reg(MCP_REG_TXB0D0 + i, data[i]);
    }

    /* Request send – RTS for TXB0 via SPI command */
    uint8_t cmd = MCP_CMD_RTS(0);
    spi_transaction_t t = {
        .length    = 8,
        .rxlength  = 0,
        .tx_buffer = &cmd,
    };
    spi_device_transmit(s_spi_dev, &t);

    xSemaphoreGive(s_spi_mutex);
    return ESP_OK;
}

esp_err_t can_read(uint32_t *id, uint8_t *data, uint8_t *len, int timeout_ms)
{
    if (id == NULL || data == NULL || len == NULL || s_spi_dev == NULL ||
        *len < 8) {
        return ESP_ERR_INVALID_ARG;
    }

    int elapsed = 0;
    int tick = 1;  /* poll interval 1 ms */

    while (1) {
        xSemaphoreTake(s_spi_mutex, portMAX_DELAY);
        uint8_t intf = mcp_read_reg(MCP_REG_CANINTF);
        uint8_t sidh_reg = 0;
        uint8_t sidl_reg = 0;
        uint8_t dlc_reg = 0;
        uint8_t data_reg = 0;
        uint8_t flag = 0;

        if ((intf & MCP_INT_RX0IF) != 0) {
            sidh_reg = MCP_REG_RXB0SIDH;
            sidl_reg = MCP_REG_RXB0SIDL;
            dlc_reg = MCP_REG_RXB0DLC;
            data_reg = MCP_REG_RXB0D0;
            flag = MCP_INT_RX0IF;
        } else if ((intf & MCP_INT_RX1IF) != 0) {
            sidh_reg = MCP_REG_RXB1SIDH;
            sidl_reg = MCP_REG_RXB1SIDL;
            dlc_reg = MCP_REG_RXB1DLC;
            data_reg = MCP_REG_RXB1D0;
            flag = MCP_INT_RX1IF;
        }

        if (flag != 0) {
            uint8_t sidh = mcp_read_reg(sidh_reg);
            uint8_t sidl = mcp_read_reg(sidl_reg);
            *id = ((uint32_t)sidh << 3) | (sidl >> 5);

            uint8_t dlc = mcp_read_reg(dlc_reg) & 0x0F;
            if (dlc > 8) dlc = 8;

            for (int i = 0; i < dlc; i++) {
                data[i] = mcp_read_reg(data_reg + i);
            }
            *len = dlc;

            mcp_bit_modify(MCP_REG_CANINTF, flag, 0x00);

            xSemaphoreGive(s_spi_mutex);
            return ESP_OK;
        }

        xSemaphoreGive(s_spi_mutex);

        if (timeout_ms >= 0 && elapsed >= timeout_ms) break;

        vTaskDelay(pdMS_TO_TICKS(tick));
        elapsed += tick;
    }

    *len = 0;
    return ESP_ERR_TIMEOUT;
}

spi_device_handle_t can_get_spi_handle(void)
{
    return s_spi_dev;
}

/* ===================================================================
 *  CAN 轮询任务 —— 每 2 秒请求远端节点温湿度数据
 *
 *  通信协议：
 *    请求帧：ID=0x101, DLC=1, Data=[0x01]（查询命令）
 *    响应帧：ID=0x201, DLC=4, Data=[TempH, TempL, HumiH, HumiL]
 *            温度/湿度均为 int16 原始值（×10），转换：实际值 = raw × 0.1
 *  超时：500ms 内无响应则记录超时告警。
 * =================================================================== */
static void can_poll_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(3000));

    while (1) {
#ifdef CONFIG_CAN_MODE_LOOPBACK
        /* ---- 回环模式：用 SHT30_B 真实数据构造远端传感器帧 ----
         * 通过 MCP2515 内部环回发送→接收，模拟 CAN 远端传输 */
        {
            int16_t raw_temp = (int16_t)(g_remote_raw_temp * 10.0f);
            int16_t raw_humi = (int16_t)(g_remote_raw_humi * 10.0f);
            uint8_t tx_data[4] = {
                (uint8_t)(raw_temp >> 8),
                (uint8_t)(raw_temp & 0xFF),
                (uint8_t)(raw_humi >> 8),
                (uint8_t)(raw_humi & 0xFF)
            };

            can_send(0x201, tx_data, 4);
            vTaskDelay(pdMS_TO_TICKS(10));

            uint32_t rx_id = 0;
            uint8_t  rx_data[8] = {0};
            uint8_t  rx_len = 8;

            esp_err_t ret = can_read(&rx_id, rx_data, &rx_len, 100);
            if (ret == ESP_OK && rx_len >= 4) {
                int16_t rt = (int16_t)((rx_data[0] << 8) | rx_data[1]);
                int16_t rh = (int16_t)((rx_data[2] << 8) | rx_data[3]);
                g_can_temp = rt * 0.1f;
                g_can_humi = rh * 0.1f;
                ESP_LOGI(TAG, "Loopback: remote data sent via CAN, ID=0x%03" PRIx32 ", T=%.1f C, H=%.1f %%",
                         rx_id, g_can_temp, g_can_humi);
            }
        }
#elif defined(CONFIG_CAN_MODE_LISTEN)
        /* 只听模式：不发送，只接收 */
        {
            uint32_t rx_id = 0;
            uint8_t  rx_data[8] = {0};
            uint8_t  rx_len = 8;
            esp_err_t ret = can_read(&rx_id, rx_data, &rx_len, 2500);
            if (ret == ESP_OK && rx_len >= 4) {
                int16_t rt = (int16_t)((rx_data[0] << 8) | rx_data[1]);
                int16_t rh = (int16_t)((rx_data[2] << 8) | rx_data[3]);
                float temp = rt * 0.1f;
                float humi = rh * 0.1f;
                if (temp > -40.0f && temp < 85.0f && humi >= 0.0f && humi <= 100.0f) {
                    g_can_temp = temp;
                    g_can_humi = humi;
                    ESP_LOGI(TAG, "Remote: ID=0x%03" PRIx32 ", T=%.1f C, H=%.1f %%", rx_id, temp, humi);
                }
            }
        }
#else
        /* ---- 正常模式：发送请求帧(0x101)，等待远端响应(0x201) ---- */
        {
            uint8_t req = 0x01;
            can_send(0x101, &req, 1);

            uint32_t rx_id = 0;
            uint8_t  rx_data[8] = {0};
            uint8_t  rx_len = 8;

            esp_err_t ret = can_read(&rx_id, rx_data, &rx_len, 2500);
            if (ret == ESP_OK && rx_len >= 4) {
                int16_t rt = (int16_t)((rx_data[0] << 8) | rx_data[1]);
                int16_t rh = (int16_t)((rx_data[2] << 8) | rx_data[3]);
                float temp = rt * 0.1f;
                float humi = rh * 0.1f;
                if (temp > -40.0f && temp < 85.0f && humi >= 0.0f && humi <= 100.0f) {
                    g_can_temp = temp;
                    g_can_humi = humi;
                    ESP_LOGI(TAG, "Remote: ID=0x%03" PRIx32 ", T=%.1f C, H=%.1f %%", rx_id, temp, humi);
                }
            } else if (ret == ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "No response from remote node (timeout)");
            }
        }
#endif
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* Start the poll task (called from main after can_init).
   Not in the header — main.c uses the weak-call convention below. */
void can_start_poll(void)
{
    xTaskCreate(can_poll_task, "can_poll", 4096, NULL, 5, &s_poll_task);
}
