#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 SSD1306 OLED 显示屏（128x64，页寻址模式）
 *
 * @param bus_handle  I2C 总线句柄
 * @param dev_handle  输出参数，返回创建的 SSD1306 设备句柄
 * @return esp_err_t  ESP_OK 成功，其他为错误码
 */
esp_err_t ssd1306_init(i2c_master_bus_handle_t bus_handle,
                       i2c_master_dev_handle_t *dev_handle);

/**
 * @brief 清空整个显示缓冲区
 */
void ssd1306_clear(void);

/**
 * @brief 在 OLED 上显示两行字符串（每行最长 16 字符，8x16 字体）
 *
 * @param line1  第一行字符串（显示在第0页，对应上方 16 像素）
 * @param line2  第二行字符串（显示在第2页，对应下方 16 像素）
 */
void ssd1306_display_string(const char *line1, const char *line2);

/**
 * @brief 诊断用：填充整个 framebuffer 并刷新。
 *
 * @param pattern  填充字节（如 0xFF=全亮, 0x55=棋盘）
 * @param delay_ms 刷新后等待的毫秒数
 */
void ssd1306_test_fill(uint8_t pattern, uint32_t delay_ms);

/**
 * @brief 获取 framebuffer 指针，供外部直接写入测试图案
 * @return 指向 1024 字节 framebuffer 的指针（128×8 pages）
 */
uint8_t *ssd1306_get_framebuffer(void);

/**
 * @brief 刷新 framebuffer 到 OLED 硬件（供外部测试用）
 */
void ssd1306_flush(void);

#ifdef __cplusplus
}
#endif

#endif /* SSD1306_H */