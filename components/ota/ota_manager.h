#ifndef OTA_MANAGER_H
#define OTA_MANAGER_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 OTA 功能
 *
 * @return esp_err_t  ESP_OK 成功，其他为错误码
 */
esp_err_t ota_init(void);

/**
 * @brief 触发 OTA 升级，从指定 URL 下载固件并写入 OTA 分区
 *
 * 下载进度每 10% 打印一次，完成后标记有效并延迟 3 秒重启。
 *
 * @param url  固件的 HTTPS URL
 * @return esp_err_t  ESP_OK 成功，其他为错误码
 */
esp_err_t ota_trigger(const char *url);

#ifdef __cplusplus
}
#endif

#endif /* OTA_MANAGER_H */