#include <Eigen/Dense>

class KalmanFilter
{
public:
    static void predict(Eigen::VectorXd &x, Eigen::MatrixXd &P, const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &B, const Eigen::VectorXd &u);
    static void update(Eigen::VectorXd &x, Eigen::MatrixXd &P, const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R);
};