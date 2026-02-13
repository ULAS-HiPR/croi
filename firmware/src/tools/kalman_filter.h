#include <Eigen/Dense>
#include "../data.h"

class KalmanFilter {
public:
    KalmanFilter();
    ~KalmanFilter();

    void predict(float time);
    void update(float z_baro, float z_accel);
    void update_values(prediction_data* data) {
        data->altitude = x(0);
        data->velocity = x(1);
        data->acceleration = x(2);
    }


private:
    Eigen::VectorXd x;  // State vector
    Eigen::VectorXd old_x; // Old state vector
    Eigen::MatrixXd F;  // State transition matrix
    Eigen::MatrixXd P;  // State covariance matrix
    Eigen::MatrixXd H;  // Measurement matrix
    Eigen::MatrixXd R;  // Measurement noise covariance matrix
    Eigen::MatrixXd Q;  // Process noise covariance matrix
};