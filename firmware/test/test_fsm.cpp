#ifndef TEST_FSM_H
#define TEST_FSM_H
#include <unity.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <stdint.h>


extern "C" void StartFSM(void* argument);

#include <tools/kalman_filter.h>
#include <tools/state_machine.h>
#include <IMU/IMU.h>
#include <Baro/baro.h>
#include <data.h>
#include <fsm.h>


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

    bool init() override { return true; }  
    bool update(baro_data* out) override {
        if (idx >= samples.size()) return false;
        *out = samples[idx++];
        return true;
    }
};


static void loadCSV(const char* path,
                    MockIMU& imu,
                    MockBaro& baro)
{
    std::ifstream file(path);
    TEST_ASSERT_TRUE(file.is_open());

    std::string line;

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
            if (!cp.checked && time >= cp.time_ms) {  
                printf("[DEBUG] Time %u ms -> FSM state = %d (checkpoint %u ms)\n", time, sm.current_state, cp.time_ms);
                fflush(stdout);
            
                cp.checked = true;  
            
                if (sm.current_state < cp.expected_state) {
                    throw std::runtime_error(
                        "FSM state failed at checkpoint " + std::to_string(cp.time_ms)
                    );
                }
            }
        }
    };

    try {
        StartFSM(&args);   
    }
    catch (const EndTest&) {
        
    }
    catch (const std::runtime_error& e) {
        TEST_FAIL_MESSAGE(e.what());
    }


    TEST_ASSERT_TRUE(sm.current_state == 5); 
}


int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_FSM_CSV);
    return UNITY_END();
}

#endif // TEST_FSM_H