#include "sht30.h"
#include <stdint.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SHT30_I2C_ADDR         0x44
#define SHT30_CMD_MEASURE_H    0x2C
#define SHT30_CMD_MEASURE_L    0x06

static const char *TAG = "sht30";

/* CRC-8: 多项式 x^8 + x^5 + x^4 + 1 (0x31)，初始值 0xFF */
static uint8_t sht30_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

esp_err_t sht30_init(i2c_master_bus_handle_t bus_handle,
                     i2c_master_dev_handle_t *dev_handle)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address   = SHT30_I2C_ADDR,
        .scl_speed_hz    = 100000,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SHT30 device: %s", esp_err_to_name(ret));
    }
    return ret;
}

/* 读取 SHT30 温湿度数据
 * 发送 0x2C06 测量命令（高重复性，clock stretching 禁用），
 * 等待 20ms 后读取 6 字节原始数据（温度2+CRC1+湿度2+CRC1）。
 * 温度转换公式：T[°C] = -45 + 175 * (Raw / 65535)
 * 湿度转换公式：H[%RH] = 100 * (Raw / 65535) */
esp_err_t sht30_read_temp_humidity(i2c_master_dev_handle_t dev_handle,
                                   float *temp, float *humidity)
{
    /* 测量命令：高重复性，clock stretching 禁用 */
    uint8_t cmd[2] = {SHT30_CMD_MEASURE_H, SHT30_CMD_MEASURE_L};
    uint8_t data[6] = {0};  /* 原始数据缓冲区 [T_MSB, T_LSB, CRC, H_MSB, H_LSB, CRC] */

    esp_err_t ret = i2c_master_transmit(dev_handle, cmd, sizeof(cmd), 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SHT30 write command failed");
        return ret;
    }

    /* 等待测量完成（SHT30 单次测量最大时间约 15ms，这里给足余量） */
    vTaskDelay(pdMS_TO_TICKS(20));

    ret = i2c_master_receive(dev_handle, data, sizeof(data), 200);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SHT30 read data failed");
        return ret;
    }

    /* CRC-8 校验：温度数据 (data[0..1]) 使用 data[2] 校验，湿度数据 (data[3..4]) 使用 data[5] 校验 */
    if (sht30_crc8(data, 2) != data[2]) {
        ESP_LOGE(TAG, "SHT30 temperature CRC check failed");
        return ESP_ERR_INVALID_CRC;
    }
    if (sht30_crc8(data + 3, 2) != data[5]) {
        ESP_LOGE(TAG, "SHT30 humidity CRC check failed");
        return ESP_ERR_INVALID_CRC;
    }

    /* 拼合原始值（大端序）并转换为物理量 */
    uint16_t raw_temp  = ((uint16_t)data[0] << 8) | data[1];
    uint16_t raw_humid = ((uint16_t)data[3] << 8) | data[4];

    /* SHT30 数据手册指定转换公式 */
    *temp     = -45.0f + 175.0f * ((float)raw_temp / 65535.0f);   /* °C */
    *humidity = 100.0f * ((float)raw_humid / 65535.0f);           /* %RH */

    return ESP_OK;
}