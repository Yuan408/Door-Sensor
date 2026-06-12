#ifndef WATCHDOG_H
#define WATCHDOG_H

typedef enum {
    WATCHDOG_TASK_SENSOR = 0,
    WATCHDOG_TASK_DISPLAY,
    WATCHDOG_TASK_NETWORK,
    WATCHDOG_TASK_COUNT
} watchdog_task_id_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化看门狗：
 *        - 启动 TWDT（超时 10 秒）
 *        - 注册三个用户任务到 TWDT
 *        - 创建高优先级监控任务
 */
void watchdog_init(void);

/**
 * @brief 报告指定任务存活（更新心跳计数器）
 *
 * @param task_id  任务标识，取值 WATCHDOG_TASK_SENSOR / DISPLAY / NETWORK
 */
void watchdog_report_alive(int task_id);

#ifdef __cplusplus
}
#endif

#endif /* WATCHDOG_H */