#include <iostream>
#include <vector>
#include "Eigen/Dense"

Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

int main()
{
  /**
   * Compute RMSE
   */
  std::vector<Eigen::VectorXd> estimations;
  std::vector<Eigen::VectorXd> ground_truth;

  // the input list of estimations
  Eigen::VectorXd e(4);
  e << 1, 1, 0.2, 0.1;
  estimations.push_back(e);
  e << 2, 2, 0.3, 0.2;
  estimations.push_back(e);
  e << 3, 3, 0.4, 0.3;
  estimations.push_back(e);

  // the corresponding list of ground truth values
  Eigen::VectorXd g(4);
  g << 1.1, 1.1, 0.3, 0.2;
  ground_truth.push_back(g);
  g << 2.1, 2.1, 0.4, 0.3;
  ground_truth.push_back(g);
  g << 3.1, 3.1, 0.5, 0.4;
  ground_truth.push_back(g);

  // call the CalculateRMSE and print out the result
  std::cout << CalculateRMSE(estimations, ground_truth) << std::endl;

  return 0;
}

Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth)
{
    Eigen::VectorXd rmse(4);
    rmse << 0,0,0,0;

    // TODO: YOUR CODE HERE
    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size

    if ((estimations.size() > 0) && (estimations.size() == ground_truth.size()))
    {
        // TODO: accumulate squared residuals
        for (int i=0; i < estimations.size(); ++i) {
            // ... your code here
            Eigen::VectorXd residual = estimations[i] - ground_truth[i];
            residual = residual.array().pow(2);
            rmse += residual;
        }

        // TODO: calculate the mean
        rmse /= estimations.size();

        // TODO: calculate the squared root
        rmse = rmse.array().sqrt();
    }

    // return the result
    return rmse;
}
