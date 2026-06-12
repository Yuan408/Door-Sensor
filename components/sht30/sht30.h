#ifndef SHT30_H
#define SHT30_H

#include <stdint.h>
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 SHT30 传感器设备句柄
 *
 * @param bus_handle  I2C 总线句柄
 * @param dev_handle  输出参数，返回创建的 SHT30 设备句柄
 * @return esp_err_t  ESP_OK 成功，其他为错误码
 */
esp_err_t sht30_init(i2c_master_bus_handle_t bus_handle,
                     i2c_master_dev_handle_t *dev_handle);

/**
 * @brief 从 SHT30 读取温湿度（单次测量模式）
 *
 * @param dev_handle  SHT30 设备句柄
 * @param temp        输出参数，温度值（摄氏度）
 * @param humidity    输出参数，湿度值（%RH）
 * @return esp_err_t  ESP_OK 成功，其他为错误码
 */
esp_err_t sht30_read_temp_humidity(i2c_master_dev_handle_t dev_handle,
                                   float *temp, float *humidity);

#ifdef __cplusplus
}
#endif

#endif /* SHT30_H */