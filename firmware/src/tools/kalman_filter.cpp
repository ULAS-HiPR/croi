#include "kalman_filter.h"
#include <cmath>

KalmanFilter::KalmanFilter()
    : R_scalar_(R_COAST), accel_buf_(0.0f), imu_scale_(1.0f)
{
    x = Eigen::Vector3d::Zero();

    P = Eigen::Matrix3d::Zero();
    P(0,0) = 100.0;
    P(1,1) = 25.0;
    P(2,2) = 0.5;

    Q = Eigen::Matrix3d::Zero();
    Q(0,0) = Q_POS;
    Q(1,1) = Q_VEL;
    Q(2,2) = Q_BIAS;

    H << 1.0, 0.0, 0.0;
    F = Eigen::Matrix3d::Identity();
    B = Eigen::Vector3d::Zero();
}


bool KalmanFilter::is_calibrated() {
    if (calib_done_) return true;
    if (calib_count_ < MIN_CALIB_SAMPLES) return false;

    // Uses mean of per-sample magnitudes: mean(sqrt(ax²+ay²+az²))

    double mean_mag = calib_sum_mag_ / calib_count_;
    imu_scale_ = (mean_mag > 0.05) ? static_cast<float>(1.0 / mean_mag) : 1.0f;
    calib_done_ = true;
    return true;
}

//Predict
void KalmanFilter::predict(float dt) {
    const double t  = static_cast<double>(dt);
    const double t2 = t * t;

    // Bias column is NEGATIVE: corrected_accel = input - bias
    F << 1.0,  t,  -0.5 * t2,
         0.0, 1.0,        -t,
         0.0, 0.0,       1.0;
    B << 0.5 * t2, t, 0.0;

    x = F * x + B * static_cast<double>(accel_buf_);
    P = F * P * F.transpose() + Q;
}

// ── Update ────────────────────────────────────────────────────────────────────
void KalmanFilter::update(float baro_alt_agl,
                           float raw_accel_axis,
                           float raw_ax,
                           float raw_az) {
    // Accumulate calibration samples via online running mean.
    // Stops once calibration is locked (calib_done_ = true).
    if (!calib_done_) {
        double a = raw_ax, b = raw_accel_axis, c = raw_az;
        calib_sum_mag_ += std::sqrt(a*a + b*b + c*c);
        calib_count_++;
    }

    // Convert raw axis reading to specific force (m/s², up positive).
    // imu_scale_ = 1.0 until calibration locks, then the correct value.
    accel_buf_ = IMU_AXIS_SIGN * raw_accel_axis * imu_scale_ * 9.81f;

    // Kalman measurement update
    const double y = static_cast<double>(baro_alt_agl) - H * x;
    const double S = (H * P * H.transpose())(0, 0) + R_scalar_;
    const Eigen::Vector3d K = (P * H.transpose()) / S;
    x = x + K * y;

    // covariance update
    const Eigen::Matrix3d IKH = Eigen::Matrix3d::Identity() - K * H;
    P = IKH * P * IKH.transpose() + (K * R_scalar_) * K.transpose();

    // Update apogee confirmation state machine
    update_apogee_confirmation(static_cast<float>(x(1)),
                               static_cast<float>(x(0)));
}

//Apogee confirmation
void KalmanFilter::update_apogee_confirmation(float velocity, float altitude) {
    if (apogee_confirmed_) return;

    if (altitude < MIN_APOGEE_ALT_M) {
        apogee_confirm_active_   = false;
        apogee_confirm_start_ms_ = 0;
        return;
    }

    if (velocity < 0.0f) {
        if (!apogee_confirm_active_) {
            apogee_confirm_active_   = true;
            apogee_confirm_start_ms_ = HAL_GetTick();
        } else if (HAL_GetTick() - apogee_confirm_start_ms_ >= APOGEE_CONFIRM_MS) {
            apogee_confirmed_ = true;
        }
    } else {
        // Velocity went positive — baro noise spike near apogee, reset
        apogee_confirm_active_   = false;
        apogee_confirm_start_ms_ = 0;
    }
}