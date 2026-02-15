#ifndef TEST_FSM_H
#define TEST_FSM_H
#include <unity.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <stdint.h>

// =======================
// Forward declarations from firmware
// =======================

extern "C" void StartFSM(void* argument);

// Bring in your real headers
#include <tools/kalman_filter.h>
#include <tools/state_machine.h>
#include <IMU/IMU.h>
#include <Baro/baro.h>
#include <data.h>
#include <fsm.h>


// =======================
// Fake HAL + FreeRTOS
// =======================

static uint32_t fake_time = 0;

static std::function<void(uint32_t)> tick_callback;

struct EndTest {};

extern "C" {

uint32_t HAL_GetTick() {
    return fake_time;
}

void osDelay(uint32_t ms) {
    fake_time += ms;

    if (tick_callback) {
        tick_callback(fake_time);
    }

    // Stop after 122.219 seconds of simulated time
    if (fake_time > 12221900) {
        throw EndTest{};
    }
}

void SystemClock_Config() {}
void Error_Handler() {}

}

class MockIMU : public IMU {
public:
    std::vector<imu_data> samples;
    size_t idx = 0;

    bool init() override { return true; }  // <--- added
    bool update(imu_data* out) override {
        if (idx >= samples.size()) return false;
        *out = samples[idx++];
        return true;
    }
};

class MockBaro : public Baro {
public:
    std::vector<baro_data> samples;
    size_t idx = 0;

    bool init() override { return true; }  // <--- added
    bool update(baro_data* out) override {
        if (idx >= samples.size()) return false;
        *out = samples[idx++];
        return true;
    }
};

// =======================
// CSV Loader
// Format:
// time_ms, accel_y, altitude
// =======================

static void loadCSV(const char* path,
                    MockIMU& imu,
                    MockBaro& baro)
{
    std::ifstream file(path);
    TEST_ASSERT_TRUE(file.is_open());

    std::string line;

    // Skip header
    std::getline(file, line);

std::vector<uint32_t> sample_times;

while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string token;

    float time_sec, altitude, velocity, accel_y;

    std::getline(ss, token, ','); time_sec = std::stof(token);
    std::getline(ss, token, ','); altitude = std::stof(token);
    std::getline(ss, token, ','); velocity = std::stof(token);
    std::getline(ss, token, ','); accel_y = std::stof(token);

    imu_data imu_sample{};
    imu_sample.acceleration.y = accel_y;

    baro_data baro_sample{};
    baro_sample.altitude = altitude;

    imu.samples.push_back(imu_sample);
    baro.samples.push_back(baro_sample);

    sample_times.push_back(static_cast<uint32_t>(time_sec * 1000.0f));
}
}

// =======================
// FSM Task Args (same as firmware)
// =======================


// =======================
// The Actual Test
// =======================

void test_FSM_CSV(void)
{
    MockIMU imu;
    MockBaro baro;

    printf("Loading CSV from path: test/data/EuRoC24.csv\n");
    fflush(stdout);
    loadCSV("test/data/EuRoC24.csv", imu, baro);

    printf("Loaded %zu IMU samples and %zu Baro samples\n", imu.samples.size(), baro.samples.size());
    fflush(stdout); 
    KalmanFilter kalman;

    flash_internal_data settings {
        .main_height = 200,
        .drouge_delay = 0,
        .liftoff_thresh = 20
    };

    StateMachine sm(settings);

    FSM_TaskArgs args;
    args.imu = &imu;
    args.baro = &baro;
    args.kalman = &kalman;
    args.state_machine = &sm;
    args.settings = settings;

    fake_time = 0;
    struct Checkpoint { uint32_t time_ms; int expected_state; bool checked = false; };
    // need to figure out mapping
    std::vector<Checkpoint> checkpoints = {
        {0, 1, false},
        {56900, 2, false},
        {636000, 4, false},
        {11082000, 5, false},
    };

    tick_callback = [&](uint32_t time) {
        for (auto& cp : checkpoints) {
            if (!cp.checked && time >= cp.time_ms) {  // use >= instead of ==
                printf("[DEBUG] Time %u ms -> FSM state = %d (checkpoint %u ms)\n", time, sm.current_state, cp.time_ms);
                fflush(stdout);
            
                cp.checked = true;  // mark checkpoint as done
            
                if (sm.current_state < cp.expected_state) {
                    throw std::runtime_error(
                        "FSM state failed at checkpoint " + std::to_string(cp.time_ms)
                    );
                }
            }
        }
    };

    try {
        StartFSM(&args);   // run your real FSM task
    }
    catch (const EndTest&) {
        // normal end of simulation
    }
    catch (const std::runtime_error& e) {
        // report to Unity explicitly
        TEST_FAIL_MESSAGE(e.what());
    }

    // Example assertion — adjust to your enum

    TEST_ASSERT_TRUE(sm.current_state == 5); // eg: past apogee
}


int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_FSM_CSV);
    return UNITY_END();
}

#endif // TEST_FSM_H