#include "kalman_filter.hpp"

void KalmanFilter::predict(Eigen::VectorXd &x, Eigen::MatrixXd &P, const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &B, const Eigen::VectorXd &u)
{
    x = F * x + B * u;
    P = F * P * F.transpose() + Q;
}

void KalmanFilter::update(Eigen::VectorXd &x, Eigen::MatrixXd &P, const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R)
{
    Eigen::VectorXd y = z - (H * x);
    Eigen::MatrixXd S = (H * P * H.transpose()) + R;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();
    
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(P.rows(), P.cols());

    x = x + K * y;
    P = (I - (K * H)) * P;
}