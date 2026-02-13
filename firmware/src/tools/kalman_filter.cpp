#include "kalman_filter.h"

KalmanFilter::KalmanFilter() {
    // Initialize matrices
    x = Eigen::VectorXd(3);
    x(0, 0) = 0; // Initial position
    x(1, 0) = 0; // Initial velocity
    x(2, 0) = 0; // Initial acceleration

    // confidence in x matrix prediction
    P = Eigen::MatrixXd(3, 3);
    P << 1, 0, 0,
         0, 0.001, 0,
         0, 0, 0.0001;

    F = Eigen::MatrixXd(3, 3);

    //confidence in f matrix prediction , could scale for time in future
    Q = Eigen::MatrixXd(3, 3);
    Q << 0.001, 0, 0,
         0, 0.001, 0,
         0, 0, 100;

    H = Eigen::MatrixXd(2, 3);
    H << 1, 0, 0,
         0, 0, 1;

    R = Eigen::MatrixXd(2, 2);
    R << 0.1, 0, //baro noise
         0, 0.01; //accel noise
     
}
KalmanFilter::~KalmanFilter() {
    // Destructor
}

void KalmanFilter::predict(float t) {
    // Predict the state
    // based of s = ut + 0.5at^2, v = u + at, a = constant

    F << 1, t, 0.5 * t * t,
         0, 1, t,
         0, 0, 1;

    x = F * x; 
    P = F * P * F.transpose() + Q;
}

void KalmanFilter::update(float z_baro, float z_accel) {
    Eigen::VectorXd z(2);
    z(0) = z_baro; // barometer measurement
    z(1) = z_accel; // accelerometer measurement

    Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

    x = x + K * (z - H * x);
    P = (Eigen::MatrixXd::Identity(3, 3) - K * H) * P * (Eigen::MatrixXd::Identity(3, 3) - K * H).transpose() + K * R * K.transpose(); 
}

