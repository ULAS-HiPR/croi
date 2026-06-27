#pragma once
#include <cstdint>
#if F4
#include "stm32f4xx_hal.h"
#include "platform/stm_f4.h"
#endif
#include "cmsis_os.h"
#include <cstdio>

#include <data.h>
#include <Flash/FlashLogger.h>
#include <Flash/flash.h>

#define LOGGER_DELAY_MS 100
#define LOGGER_POST_LANDING_MS 60000U
#define LOGGER_FLASH_START_ADDRESS 0x00000000U
#define LOGGER_FLASH_LENGTH 0x01000000U

enum class LoggerFault : uint8_t {
    None = 0,
    NoStorage,
    FlashInitFailed,
    LogBeginFailed,
    Full,
    Corrupt,
    FlashIo,
    VerifyFailed,
    PayloadTooLarge,
    NotInitialized,
    Unknown
};

struct LoggerHealth {
    volatile bool initialized{false};
    volatile bool preflight_ok{false};
    volatile bool fault_latched{false};
    volatile bool logging_stopped{false};
    volatile LoggerFault fault{LoggerFault::None};
    volatile FlashLogStatus flash_status{FlashLogStatus::NotInitialized};
    volatile uint32_t run_id{0U};
    volatile uint32_t records_written{0U};
};

namespace task{
class Logger {
    public:
        Logger(Flash* storage, osMessageQueueId_t logger_queue,
               osMessageQueueId_t reciver_queue,
               LoggerHealth* health = nullptr) :
                storage_(storage),
                logger_queue_(logger_queue),
                reciver_queue_(reciver_queue),
                health_(health == nullptr ? &owned_health_ : health),
                taskHandle_(nullptr)
        {};

        void run();
        const LoggerHealth* health() const { return health_; }

        struct LogMessage {
            uint32_t timestamp;
            imu_data imu;
            //add in for croi
        };

    private:
        void StartLogger();
        static void StartLoggerEntry(void *argument);
        bool configure_logger(FlashLogger& flash_logger);
        bool log_flight_data(FlashLogger& flash_logger, const flight_data& data);
        bool log_secondary_data(FlashLogger& flash_logger,
                                const secondary_flight_data& data);
        void latch_fault(LoggerFault fault, FlashLogStatus status);
        void update_health_ok(const FlashLogger& flash_logger);
        static bool preflight_state(State state);
        static LoggerFault map_flash_status(FlashLogStatus status);
        uint32_t endlog_time{0};
        bool logging_stop_timer{true};
        bool is_logger_active(State state, uint32_t time);
       

        Flash* storage_;
        osMessageQueueId_t logger_queue_;
        osMessageQueueId_t reciver_queue_;
        LoggerHealth owned_health_{};
        LoggerHealth* health_;
        osThreadId_t taskHandle_;

        const osThreadAttr_t task_attributes {
            "Logger",
            0,
            nullptr,
            0,
            nullptr,
            1024,      
            osPriorityLow,
            0,
            0
        };

    };

}