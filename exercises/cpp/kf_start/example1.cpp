#include <Eigen/Dense>
#include <vector>
#include <iostream>

#include "kalman_filter.hpp"

int main()
{
    Eigen::VectorXd x(2);     // state vector
    Eigen::MatrixXd P(2, 2);  // state covariance
    Eigen::MatrixXd F(2, 2);  // state transition matrix
    Eigen::MatrixXd Q(2, 2);  // process noise covariance
    Eigen::VectorXd u(1);     // control input vector
    Eigen::MatrixXd B(2, 1);  // input transition matrix
    Eigen::MatrixXd H(1, 2);  // measurement transition matrix
    Eigen::MatrixXd R(1, 1);  // measurement noise covariance

    x << 0.0, 0.0;
    P << 1000.0 , 0.0, 0.0, 1000.0;
    F << 1.0, 1.0, 0.0, 1.0;
    Q << 0.0, 0.0, 0.0, 0.0;
    u << 0.0;
    B << 0.0, 0.0;
    H << 1.0, 0.0;
    R << 1.0;

    std::vector<Eigen::VectorXd> measurements;
    Eigen::VectorXd singleMeas(1);

    singleMeas << 1;
    measurements.push_back(singleMeas);
    singleMeas << 2;
    measurements.push_back(singleMeas);
    singleMeas << 3;
    measurements.push_back(singleMeas);
    
    for (Eigen::VectorXd z : measurements)
    {
        KalmanFilter::predict(x, P, F, Q, B, u);
        KalmanFilter::update(x, P, z, H, R);
        
        std::cout << "x =   " << x << std::endl;
        std::cout << "P =   " << P << std::endl;
    }
}
