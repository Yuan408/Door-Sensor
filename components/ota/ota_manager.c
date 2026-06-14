#include "ota_manager.h"
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"

static const char *TAG = "ota_manager";
static bool s_ota_in_progress = false;

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

static void ota_task(void *arg)
{
    char *url = (char *)arg;

    ESP_LOGI(TAG, "Starting OTA from URL: %s", url);

    esp_http_client_config_t http_cfg = {
        .url = url,
        .skip_cert_common_name_check = true,
        .timeout_ms = 30000,
        .keep_alive_enable = true,
    };

    esp_https_ota_config_t ota_cfg = {
        .http_config = &http_cfg,
    };

    ESP_LOGI(TAG, "OTA download in progress, please wait...");
    esp_err_t ret = esp_https_ota(&ota_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OTA upgrade failed: %s", esp_err_to_name(ret));
        s_ota_in_progress = false;
        free(url);
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "OTA upgrade successful! Rebooting in 3 seconds...");
    free(url);
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_restart();
}

esp_err_t ota_trigger(const char *url)
{
    if (url == NULL) {
        ESP_LOGE(TAG, "OTA URL is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (s_ota_in_progress) {
        ESP_LOGW(TAG, "OTA already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    char *url_copy = strdup(url);
    if (url_copy == NULL) {
        ESP_LOGE(TAG, "Failed to allocate OTA URL buffer");
        return ESP_ERR_NO_MEM;
    }

    s_ota_in_progress = true;
    BaseType_t created = xTaskCreate(ota_task, "ota_task", 8192, url_copy, 5, NULL);
    if (created != pdPASS) {
        s_ota_in_progress = false;
        free(url_copy);
        ESP_LOGE(TAG, "Failed to create OTA task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA task started");
    return ESP_OK;
}
