#include <vector>
#include <iostream>
#include <Eigen/Dense>

Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd &x_state);

int main(int argc, char **argv)
{
    ///
    /// Compute the Jacobian Matrix
    ///

    // predicted state example
    // px = 1, py = 2, vx = 0.2, vy = 0.4
    Eigen::VectorXd x_predicted(4);
    x_predicted << 1, 2, 0.2, 0.4;

    Eigen::MatrixXd Hj = CalculateJacobian(x_predicted);

    std::cout << "Hj:" << std::endl << Hj << std::endl;

    return 0;
}

Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd &x_state)
{
    Eigen::MatrixXd J(3, x_state.rows());

    const float px = x_state(0);
    const float py = x_state(1);
    const float vx = x_state(2);
    const float vy = x_state(3);

    if (std::abs(px) < std::numeric_limits<float>::epsilon() && std::abs(py) < std::numeric_limits<float>::epsilon())
    {
        std::cout << "Division by Zero Error" << "\n";
        J.setZero();
    }
    else
    {
        float a = px*px + py*py;
        float b = std::sqrt(a);
        float c = std::pow(a, 1.5);

        J(0, 0) = px / b;
        J(0, 1) = py / b;
        J(0, 2) = 0.0;
        J(0, 3) = 0.0;

        J(1, 0) = -py / a;
        J(1, 1) = px / a;
        J(1, 2) = 0.0;
        J(1, 3) = 0.0;

        J(2, 0) = py*(vx*py - vy*px) / c;
        J(2, 1) = px*(vy*px - vx*py) / c;
        J(2, 2) = px / b;
        J(2, 3) = py / b;
    }

    return J;
}