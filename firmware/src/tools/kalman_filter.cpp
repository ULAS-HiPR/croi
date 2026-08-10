#include "kalman_filter.h"

KalmanFilter::KalmanFilter()
    : R_scalar_(R_COAST), accel_buf_(0.0f)
{
    x[0] = 0.0f; x[1] = 0.0f; x[2] = 0.0f;

    mat3_zero(P);
    P[0][0] = 100.0f;
    P[1][1] = 25.0f;
    P[2][2] = 0.5f;

    mat3_zero(Q);
    Q[0][0] = Q_POS;
    Q[1][1] = Q_VEL;
    Q[2][2] = Q_BIAS;

    H[0] = 1.0f; H[1] = 0.0f; H[2] = 0.0f;
}

void KalmanFilter::predict(float dt) {
    const float t2 = dt * dt;

    float F[3][3] = {
        {1.0f,  dt,  -0.5f * t2},
        {0.0f, 1.0f,        -dt},
        {0.0f, 0.0f,       1.0f}
    };
    float B[3] = {0.5f * t2, dt, 0.0f};

    float xn[3];
    mat3_vec_mul(F, x, xn);
    xn[0] += B[0] * accel_buf_;
    xn[1] += B[1] * accel_buf_;
    x[0] = xn[0]; x[1] = xn[1]; x[2] = xn[2];

    float FP[3][3], FPFt[3][3];
    mat3_mul(F, P, FP);
    mat3_mul_transpose(FP, F, FPFt);
    mat3_add(FPFt, Q, P);
}

void KalmanFilter::update(float baro_alt,
                           float raw_accel_axis,
                           float raw_ax,
                           float raw_az) {
    if (!calib_done_) {
        float a = raw_ax, b = raw_accel_axis, c = raw_az;
        calib_sum_mag_ += sqrtf(a*a + b*b + c*c);
        calib_count_++;
    }

    accel_buf_ = raw_accel_axis;

    const float y = baro_alt - (H[0]*x[0] + H[1]*x[1] + H[2]*x[2]);
    const float S = P[0][0] + R_scalar_;

    float K[3];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;
    K[2] = P[2][0] / S;

    x[0] += K[0] * y;
    x[1] += K[1] * y;
    x[2] += K[2] * y;

    float IKH[3][3] = {
        {1.0f - K[0]*H[0], 0.0f, 0.0f},
        {     - K[1]*H[0], 1.0f, 0.0f},
        {     - K[2]*H[0], 0.0f, 1.0f}
    };

    float IKH_P[3][3], newP[3][3];
    mat3_mul(IKH, P, IKH_P);
    mat3_mul_transpose(IKH_P, IKH, newP);

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            newP[i][j] += K[i] * R_scalar_ * K[j];

    mat3_copy(newP, P);
}

bool KalmanFilter::is_calibrated() {
    if (calib_done_) return true;
    if (calib_count_ < MIN_CALIB_SAMPLES) return false;
    float mean_mag = calib_sum_mag_ / calib_count_;
    imu_scale_ = (mean_mag > 0.05f) ? (1.0f / mean_mag) : 1.0f;
    calib_done_ = true;
    return true;
}

void KalmanFilter::mat3_zero(float M[3][3]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            M[i][j] = 0.0f;
}

void KalmanFilter::mat3_copy(const float src[3][3], float dst[3][3]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            dst[i][j] = src[i][j];
}

void KalmanFilter::mat3_mul(const float A[3][3], const float B[3][3], float C[3][3]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            C[i][j] = 0.0f;
            for (int k = 0; k < 3; k++)
                C[i][j] += A[i][k] * B[k][j];
        }
}

void KalmanFilter::mat3_mul_transpose(const float A[3][3], const float B[3][3], float C[3][3]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            C[i][j] = 0.0f;
            for (int k = 0; k < 3; k++)
                C[i][j] += A[i][k] * B[j][k];
        }
}

void KalmanFilter::mat3_add(const float A[3][3], const float B[3][3], float C[3][3]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            C[i][j] = A[i][j] + B[i][j];
}

void KalmanFilter::mat3_vec_mul(const float A[3][3], const float v[3], float out[3]) {
    for (int i = 0; i < 3; i++) {
        out[i] = 0.0f;
        for (int j = 0; j < 3; j++)
            out[i] += A[i][j] * v[j];
    }
}