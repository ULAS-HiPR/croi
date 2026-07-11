#include "watchdog_task.h"

#include "croi_status.h"
#include "platform/error_handler.h"
#include "platform/hal_time.h"

#if F0
#include "stm32f0xx_hal.h"
#endif

namespace task {

namespace {
constexpr uint32_t kBootGraceMs = 120000U;
constexpr uint32_t kTaskFreshnessMs = 2000U;
constexpr uint32_t kCheckPeriodMs = 250U;

bool heartbeat_fresh(uint32_t heartbeat_ms, uint32_t now_ms) {
    return heartbeat_ms != 0U &&
           static_cast<uint32_t>(now_ms - heartbeat_ms) <= kTaskFreshnessMs;
}
}

bool WatchdogTask::run() {
    task_handle_ = osThreadNew(&WatchdogTask::entry, this, &task_attributes_);
    return task_handle_ != nullptr;
}

void WatchdogTask::entry(void* argument) {
    auto* self = static_cast<WatchdogTask*>(argument);
    if (self != nullptr) {
        self->loop();
    }
}

void WatchdogTask::loop() {
#if F0
#if defined(__HAL_DBGMCU_FREEZE_IWDG)
    __HAL_DBGMCU_FREEZE_IWDG();
#endif
    IWDG_HandleTypeDef watchdog{};
    watchdog.Instance = IWDG;
    watchdog.Init.Prescaler = IWDG_PRESCALER_64;
    watchdog.Init.Reload = 2499U;
    watchdog.Init.Window = IWDG_WINDOW_DISABLE;
    if (HAL_IWDG_Init(&watchdog) != HAL_OK) {
        Error_Handler();
    }
    croi_status.watchdog_init_ok = 1U;
    const uint32_t started_ms = HAL_GetTick();
    bool supervision_started = false;

    for (;;) {
        const uint32_t now_ms = HAL_GetTick();
        const bool boot_grace = static_cast<uint32_t>(now_ms - started_ms) < kBootGraceMs;
        const bool tasks_healthy =
            heartbeat_fresh(croi_status.fsm_task_heartbeat_ms, now_ms) &&
            heartbeat_fresh(croi_status.can_task_heartbeat_ms, now_ms) &&
            heartbeat_fresh(croi_status.logger_task_heartbeat_ms, now_ms);
        supervision_started = supervision_started || tasks_healthy;
        if ((!supervision_started && boot_grace) || tasks_healthy) {
            if (HAL_IWDG_Refresh(&watchdog) == HAL_OK) {
                ++croi_status.watchdog_refresh_count;
            }
        } else {
            ++croi_status.watchdog_missed_count;
        }
        osDelay(kCheckPeriodMs);
    }
#else
    Error_Handler();
#endif
}

}
