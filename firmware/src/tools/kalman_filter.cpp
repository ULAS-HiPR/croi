#include "kalman_filter.h"


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

    float mean_mag = calib_sum_mag_ / calib_count_;
    imu_scale_ = (mean_mag > 0.05) ? (1.0 / mean_mag) : 1.0f;
    calib_done_ = true;
    return true;
}

void KalmanFilter::predict(float dt) {
    const float t  = dt;
    const float t2 = t * t;

    // Bias column is NEGATIVE: corrected_accel = input - bias
    F << 1.0,  t,  -0.5 * t2,
         0.0, 1.0,        -t,
         0.0, 0.0,       1.0;
    B << 0.5 * t2, t, 0.0;

    x = F * x + B * static_cast<double>(accel_buf_);
    P = F * P * F.transpose() + Q;
}

void KalmanFilter::update(float baro_alt_agl,
                           float raw_accel_axis,
                           float raw_ax,
                           float raw_az) {
    // Accumulate calibration samples via online running mean.
    // Stops once calibration is locked (calib_done_ = true).
    if (!calib_done_) {
        float a = raw_ax, b = raw_accel_axis, c = raw_az;
        calib_sum_mag_ += std::sqrt(a*a + b*b + c*c);
        calib_count_++;
    }

    // Convert raw axis reading to specific force (m/s², up positive).
    // imu_scale_ = 1.0 until calibration locks, then the correct value.
    accel_buf_ = IMU_AXIS_SIGN * raw_accel_axis * imu_scale_ * 9.81f;

    // Kalman measurement update
    const float y = static_cast<double>(baro_alt_agl) - H * x;
    const float S = (H * P * H.transpose())(0, 0) + R_scalar_;
    const Eigen::Vector3d K = (P * H.transpose()) / S;
    x = x + K * y;

    // covariance update
    const Eigen::Matrix3d IKH = Eigen::Matrix3d::Identity() - K * H;
    P = IKH * P * IKH.transpose() + (K * R_scalar_) * K.transpose();
}

