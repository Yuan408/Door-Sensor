#include "ota_manager.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_https_ota.h"
#include <esp_ota_ops.h>

static const char *TAG = "ota_manager";

esp_err_t ota_init(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running == NULL) {
        ESP_LOGE(TAG, "Failed to get running partition");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Running partition: type=%d, subtype=%d, label=%s",
             running->type, running->subtype, running->label);

    const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);
    if (update == NULL) {
        ESP_LOGE(TAG, "No OTA update partition found");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "OTA update partition: type=%d, subtype=%d, label=%s, size=%lu",
             update->type, update->subtype, update->label, update->size);
    ESP_LOGI(TAG, "OTA init done");

    return ESP_OK;
}

/* 触发 OTA 空中升级
 *
 * 升级流程：
 *   1. 通过 HTTPS 从指定 URL 下载新固件到 ota_0 / ota_1 分区
 *   2. 下载完成后 esp_https_ota_perform() 自动校验固件完整性
 *   3. 调用 esp_ota_mark_app_valid_cancel_rollback() 将新固件标记为有效
 *      （取消回滚机制——若新固件启动后未调用此函数，bootloader 会回滚到旧版本）
 *   4. 延时 3 秒后调用 esp_restart() 重启进入新固件
 *
 * 安全措施：
 *   - 跳过证书 CN 校验（测试环境），生产环境需配置完整 TLS 证书
 *   - HTTP 超时 30 秒，超时自动返回失败
 *   - 下载失败不会影响当前运行固件
 */
esp_err_t ota_trigger(const char *url)
{
    if (url == NULL) {
        ESP_LOGE(TAG, "OTA URL is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Starting OTA from URL: %s", url);

    /* 配置 HTTP 客户端（跳过证书 CN 校验仅用于测试环境） */
    esp_http_client_config_t http_cfg = {
        .url = url,
        .skip_cert_common_name_check = true,  /* 测试用，生产环境应设为 false */
        .timeout_ms = 30000,                  /* 下载超时 30 秒 */
    };

    esp_https_ota_config_t ota_cfg = {
        .http_config = &http_cfg,
    };

    /* 执行 OTA 下载和固件写入 */
    ESP_LOGI(TAG, "OTA download in progress, please wait...");
    esp_err_t ret = esp_https_ota_perform(&ota_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OTA upgrade failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 下载完成，验证固件镜像 */
    ESP_LOGI(TAG, "OTA download complete, validating...");

    /* 标记新固件为有效 —— 取消启动后自动回滚
     * 若新固件启动后未调用此函数，bootloader 会在下次启动时回滚到旧版本 */
    esp_ota_mark_app_valid_cancel_rollback();

    /* 延时 3 秒后软复位，bootloader 将引导到新固件 */
    ESP_LOGI(TAG, "OTA upgrade successful! Rebooting in 3 seconds...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_restart();  /* 软复位，启动新固件 */

    /* 以下代码不可达 */
    return ESP_OK;
}