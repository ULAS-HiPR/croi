#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H
#include <data.h>
#include <cmath>
#include <cstdint>

class KalmanFilter {
public:
    KalmanFilter();
    ~KalmanFilter() = default;

    void predict(float dt);
    void update(float baro_alt_agl,
                float vertical_accel_m_s2,
                float accel_x_g,
                float accel_y_g,
                float accel_z_g);

    void update_values(prediction_data* data) {
        data->altitude     = x[0];
        data->velocity     = x[1];
        data->acceleration = accel_buf_ - x[2];
    }

    void set_phase_ready()        { R_scalar_ = R_COAST;  }
    void set_phase_thrusting()    { R_scalar_ = R_THRUST; }
    void set_phase_coasting()     { R_scalar_ = R_COAST;  }
    void set_phase_drogue()       { R_scalar_ = R_DROGUE; }
    void set_phase_main()         { R_scalar_ = R_MAIN;   }

    bool is_calibrated();
    bool is_apogee_confirmed() const { return apogee_confirmed_; }

    void reset_apogee_confirmation() {
        apogee_confirm_active_   = false;
        apogee_confirm_start_ms_ = 0;
        apogee_confirmed_        = false;
    }

private:
    float x[3];
    float P[3][3];
    float Q[3][3];
    float H[3];

    float R_scalar_;
    float accel_buf_;

    static constexpr int   MIN_CALIB_SAMPLES  = 50;
    float    calib_sum_mag_ = 0.0f;
    float    calib_sum_mag_sq_ = 0.0f;
    float    calib_sum_vertical_ = 0.0f;
    int      calib_count_   = 0;
    bool     calib_done_    = false;

    static constexpr uint32_t APOGEE_CONFIRM_MS = 300;
    static constexpr float    MIN_APOGEE_ALT_M  = 100.0f;

    uint32_t apogee_confirm_start_ms_ = 0;
    bool     apogee_confirm_active_   = false;
    bool     apogee_confirmed_        = false;

    static constexpr float Q_POS    = 0.1f;
    static constexpr float Q_VEL    = 0.5f;
    static constexpr float Q_BIAS   = 1e-4f;
    static constexpr float R_COAST  = 0.3f;
    static constexpr float R_THRUST = 500.0f;
    static constexpr float R_DROGUE = 10.0f;
    static constexpr float R_MAIN   = 2.0f;

    static void mat3_zero(float M[3][3]);
    static void mat3_copy(const float src[3][3], float dst[3][3]);
    static void mat3_mul(const float A[3][3], const float B[3][3], float C[3][3]);
    static void mat3_mul_transpose(const float A[3][3], const float B[3][3], float C[3][3]);
    static void mat3_add(const float A[3][3], const float B[3][3], float C[3][3]);
    static void mat3_vec_mul(const float A[3][3], const float v[3], float out[3]);
};

#endif // KALMAN_FILTER_H
