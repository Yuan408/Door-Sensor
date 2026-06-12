#ifndef WIFI_STA_H
#define WIFI_STA_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 WiFi Station 模式，连接到 menuconfig 中配置的 AP
 *
 * 使用 CONFIG_WIFI_SSID 和 CONFIG_WIFI_PASSWORD 宏获取凭据，
 * 通过事件组等待连接成功，超时 30 秒。
 * 自动注册断线重连和 IP 获取回调。
 *
 * @return esp_err_t  ESP_OK 成功，其他为错误码
 */
esp_err_t wifi_init_sta(void);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_STA_H */