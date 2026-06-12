#include "watchdog.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include <inttypes.h>
#include "esp_task_wdt.h"

static const char *TAG = "watchdog";

/* 心跳计数器，由 watchdog_report_alive() 更新 */
static volatile uint32_t s_heartbeat[WATCHDOG_TASK_COUNT] = {0};

/* 监控任务句柄 */
static TaskHandle_t s_monitor_task_handle = NULL;

/* 标记 TWDT 是否成功初始化（用于防止重复初始化 + 控制 esp_task_wdt_reset 调用） */
static bool s_twdt_ready = false;

/* 心跳监控任务：每 3 秒遍历所有受监控的逻辑任务
 * 检查逻辑：
 *   1. 计算当前 tick 与上次心跳的差值（处理 tick 计数器的 32 位回绕）
 *   2. 若差值 > 10 秒阈值，则该任务已"假死"，触发系统软复位
 *   3. 正常情况所有任务应在 2 秒主循环周期内更新心跳 */
static void watchdog_monitor_task(void *arg)
{
    const uint32_t timeout_ticks = pdMS_TO_TICKS(10000);  /* 超时阈值：10 秒 */

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(3000));  /* 每 3 秒检查一次 */

        uint32_t now = xTaskGetTickCount();
        const char *task_names[] = {"SENSOR", "DISPLAY", "NETWORK"};

        for (int i = 0; i < WATCHDOG_TASK_COUNT; i++) {
            uint32_t elapsed;

            /* 计算距上次心跳的 tick 差值（考虑 32 位无符号计数器回绕） */
            if (now >= s_heartbeat[i]) {
                elapsed = now - s_heartbeat[i];
            } else {
                /* FreeRTOS tick 计数器为 32 位，溢出后从 0 重新计数 */
                elapsed = (UINT32_MAX - s_heartbeat[i]) + now + 1;
            }

            /* 心跳超时：该任务可能死锁或长时间阻塞，触发系统复位 */
            if (elapsed > timeout_ticks) {
                ESP_LOGE(TAG, "Task %s heartbeat timeout (elapsed=%lu ticks), restarting!",
                         task_names[i], elapsed);
                esp_restart();  /* 软件复位 */
            }
        }
    }
}

/* 初始化看门狗系统
 *
 * 双层防护机制：
 *   Layer 1 — TWDT（Task Watchdog Timer）：硬件看门狗，超时 10 秒
 *            触发条件：app_main 主循环超过 10 秒未调用 esp_task_wdt_reset()
 *            处理方式：触发 panic 并打印调用栈
 *
 *   Layer 2 — 心跳监控任务：软件层面的任务存活检测
 *            监控 3 个逻辑任务（传感器/显示/网络），每个任务必须定期
 *            调用 watchdog_report_alive() 更新心跳时间戳。心跳超时 10 秒
 *            未更新则调用 esp_restart() 软复位。
 *
 * 监控任务以最高优先级运行（configMAX_PRIORITIES - 1），确保在业务逻辑
 * 死锁时仍能正常检测和恢复。
 */
void watchdog_init(void)
{
    /* 防止重复初始化 */
    if (s_twdt_ready) {
        ESP_LOGW(TAG, "Watchdog already initialized, skipping");
        return;
    }

    /* 初始化 TWDT（硬件任务看门狗），超时 10 秒
     * 注意：若 sdkconfig 中 CONFIG_ESP_TASK_WDT_INIT=y，IDF 会在启动时自动初始化
     * TWDT，此时 esp_task_wdt_init 返回 ESP_ERR_INVALID_STATE，属于正常情况，
     * 跳过初始化继续注册任务即可。 */
    const esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 10000,        /* 10 秒无喂狗即触发 */
        .idle_core_mask = 0,        /* 不监控 idle 任务 */
        .trigger_panic = true,      /* 超时触发 panic 并打印调用栈 */
    };
    esp_err_t ret = esp_task_wdt_init(&twdt_config);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "TWDT already initialized by IDF, reusing existing TWDT");
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init TWDT: %s", esp_err_to_name(ret));
        return;
    } else {
        ESP_LOGI(TAG, "TWDT initialized (timeout=%" PRIu32 " ms)", twdt_config.timeout_ms);
    }

    /* 将当前任务（app_main 所在的主循环任务）注册到 TWDT */
    ret = esp_task_wdt_add(NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add current task to TWDT: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Current task added to TWDT");

    s_twdt_ready = true;  /* 标记 TWDT 就绪，允许 watchdog_report_alive 调用 esp_task_wdt_reset */

    /* 初始化所有逻辑任务的心跳计数器为当前 tick */
    uint32_t now = xTaskGetTickCount();
    for (int i = 0; i < WATCHDOG_TASK_COUNT; i++) {
        s_heartbeat[i] = now;
    }

    /* 创建最高优先级的监控任务，确保即使低优先级任务死锁也能正常检测 */
    BaseType_t created = xTaskCreate(
        watchdog_monitor_task,
        "watchdog_mon",
        4096,                                   /* 栈大小 4 KB */
        NULL,
        configMAX_PRIORITIES - 1,               /* 最高优先级 */
        &s_monitor_task_handle
    );
    if (created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create monitor task");
    } else {
        ESP_LOGI(TAG, "Monitor task created (priority=%d)", configMAX_PRIORITIES - 1);
    }
}

void watchdog_report_alive(int task_id)
{
    if (task_id >= 0 && task_id < WATCHDOG_TASK_COUNT) {
        s_heartbeat[task_id] = xTaskGetTickCount();
    }

    /* 仅在 TWDT 初始化/注册成功后喂狗，避免 "task not found" 错误 */
    if (s_twdt_ready) {
        esp_task_wdt_reset();
    }
}