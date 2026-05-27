#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H
#define EIGEN_NO_DEBUG
#define EIGEN_MPL2_ONLY
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#include <Eigen/Dense>
#include <data.h>
#include <cmath>
#include <cstdint>

//IMU axis configuration
#define IMU_VERTICAL_AXIS  1        // 0=Ax, 1=Ay, 2=Az
#define IMU_AXIS_SIGN      (-1.0f)  // -1.0f if axis points DOWN at rest


class KalmanFilter {
public:
    KalmanFilter();
    ~KalmanFilter() = default;


    void predict(float dt);

    void update(float baro_alt_agl,
                float raw_accel_axis,   
                float raw_ax,           
                float raw_az);          


    void update_values(prediction_data* data) {
        data->altitude     = static_cast<float>(x(0));
        data->velocity     = static_cast<float>(x(1));
        // Net vertical acceleration = corrected_SF - g
        // Positive during boost, negative during coast. Use for canard control.
        data->acceleration = accel_buf_ - static_cast<float>(x(2)) - 9.81f;
    }

    // ── R gain scheduling — call from change_state() ──────────────────────────
    void set_phase_ready()        { R_scalar_ = R_COAST;  }
    void set_phase_thrusting()    { R_scalar_ = R_THRUST; }
    void set_phase_coasting()     { R_scalar_ = R_COAST;  }
    void set_phase_drogue()       { R_scalar_ = R_DROGUE; }
    void set_phase_main()         { R_scalar_ = R_MAIN;   }

    // ── Status queries for state machine ──────────────────────────────────────

    // Call from check_calibrating_state_done().
    // Returns false until MIN_CALIB_SAMPLES have been collected.
    // On the first call that returns true, the scale is locked permanently.
    // Collect samples by calling update() during CALIBRATING state as normal.
    bool is_calibrated();

    // Returns true when KF velocity has been continuously negative for
    // APOGEE_CONFIRM_MS and altitude > MIN_APOGEE_ALT_M.
    // Call from check_coasting_state_done() instead of checking velocity directly.
    bool is_apogee_confirmed() const { return apogee_confirmed_; }

    // Call from change_state() on every transition.
    // Resets apogee confirmation so a false trigger doesn't latch.
    void reset_apogee_confirmation() {
        apogee_confirm_active_   = false;
        apogee_confirm_start_ms_ = 0;
        apogee_confirmed_        = false;
    }

private:
    // ── Kalman state ──────────────────────────────────────────────────────────
    Eigen::Vector3d    x;
    Eigen::Matrix3d    P;
    Eigen::Matrix3d    F;
    Eigen::Vector3d    B;
    Eigen::Matrix3d    Q;
    Eigen::RowVector3d H;
    double             R_scalar_;
    float              accel_buf_;

    // ── IMU scale — online running mean, no fixed buffer needed ───────────────
    static constexpr int MIN_CALIB_SAMPLES = 50;   // 0.5s at 100Hz minimum
    float  calib_sum_mag_ = 0.0;  // running sum of per-sample magnitudes
    int     calib_count_   = 0;    // samples accumulated so far
    float   imu_scale_     = 1.0f; // locked once is_calibrated() returns true
    bool    calib_done_    = false;

    // ── Apogee confirmation ───────────────────────────────────────────────────
    static constexpr uint32_t APOGEE_CONFIRM_MS  = 300;    // ms
    static constexpr float    MIN_APOGEE_ALT_M   = 100.0f; // m AGL

    uint32_t apogee_confirm_start_ms_ = 0;
    bool     apogee_confirm_active_   = false;
    bool     apogee_confirmed_        = false;


    // ── Tuned noise constants (from flights FL001 + FL002) ────────────────────
    static constexpr float Q_POS    = 0.1;
    static constexpr float Q_VEL    = 0.5;
    static constexpr float Q_BIAS   = 1e-4;
    static constexpr float R_COAST  = 0.3;
    static constexpr float R_THRUST = 500.0;
    static constexpr float R_DROGUE = 10.0;
    static constexpr float R_MAIN   = 2.0;
};

#endif // KALMAN_FILTER_H