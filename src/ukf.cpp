#include <iostream>
#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() 
{
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // initial state vector
    x_ = VectorXd::Zero(5);

    // initial covariance matrix
    P_ = MatrixXd::Identity(5, 5);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 2;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 5;
    
    /**
     * DO NOT MODIFY measurement noise values below.
     * These are provided by the sensor manufacturer.
     */

    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;
    
    /**
     * End DO NOT MODIFY section for measurement noise values 
     */
    
    /**
     * TODO: Complete the initialization. See ukf.h for other member properties.
     * Hint: one or more values initialized above might be wildly off...
     */

    is_initialized_ = false;
    
    // initialize the stored timestamp
    time_us_ = 0;

    // augmented state size (i.e state dim + noise dim)
    n_x_ = 5;
    n_aug_ = n_x_ + 2;

    // initialize the predicted state sigma points
    Xsig_pred_ = Eigen::MatrixXd(n_x_, 2 * n_aug_ + 1);
    // std::cout << "Xsig_pred_ size = " << Xsig_pred_.rows() << ", " << Xsig_pred_.cols() << "\n";

    // calculate the scalling factor lambda (i.e decides how far the sigma points locate from the mean of the gaussian)
    lambda_ = 3.0 - n_aug_;

    // set vector for weights
    weights_ = Eigen::VectorXd(2 * n_aug_ + 1);

    // set weights
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    for (int i = 1; i < 2 * n_aug_ + 1; ++i)
    {
        weights_(i) = 0.5 / (n_aug_ + lambda_);
    }
}

UKF::~UKF() {}

void UKF::normalizeAngle(double& angle) const
{
  angle = fmod(angle + M_PI, 2*M_PI);
  if(angle < 0)
  {
      angle += 2*M_PI;
  }
  angle -= M_PI;   
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
    /**
     * TODO: Complete this function! Make sure you switch between lidar and radar
     * measurements.
     */

    if (!is_initialized_)
    {
        if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER)
        {
            // set the state with the initial location and zero velocity
            x_(0) = meas_package.raw_measurements_(0);
            x_(1) = meas_package.raw_measurements_(1);
            P_(0, 0) = std_laspx_*std_laspx_;
            P_(1, 1) = std_laspy_*std_laspy_;
            is_initialized_ = true;
        }
        else if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR)
        {
            double rho = meas_package.raw_measurements_(0);
            double phi = meas_package.raw_measurements_(1);
            double drho = meas_package.raw_measurements_(2);

            double vx = drho * std::cos(phi);
            double vy = drho * std::sin(phi);

            x_(0) = rho * std::cos(phi);
            x_(1) = rho * std::sin(phi);
            //x_(2) = std::sqrt(vx*vx + vy*vy);
            x_(2) = vx * std::cos(phi) + vy * std::sin(phi); // for signed velocity value

            is_initialized_ = true;
        }
        else
        {
            ; // do nothing
        }

        time_us_ = meas_package.timestamp_;
        return;
    }

    // compute the time elapsed between the current and previous measurements
    // dt - expressed in seconds
    double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
    time_us_ = meas_package.timestamp_;

    this->Prediction(dt);

    if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
        this->UpdateLidar(meas_package);
    }

    if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
        this->UpdateRadar(meas_package);
    }

//    std::cout << "x_ = \n" << x_ << "\n";
//    std::cout << "P_ = \n" << P_ << "\n\n";
}

void UKF::Prediction(double delta_t)
{
    /**
     * TODO: Complete this function! Estimate the object's location. 
     * Modify the state vector, x_. Predict sigma points, the state, 
     * and the state covariance matrix.
     */

    // create augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);

    // create augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

    // create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

    // create augmented mean state
    x_aug.head(n_x_) = x_;
    x_aug(5) = 0.0;
    x_aug(6) = 0.0;
    // std::cout << "x_aug = \n" << x_aug << "\n";

    // create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(5, 5) = std_a_ * std_a_;
    P_aug(6, 6) = std_yawdd_ * std_yawdd_;
    // std::cout << "P_aug = \n" << P_aug << "\n";

    // create square root matrix
    Eigen::MatrixXd L_aug = P_aug.llt().matrixL();
    // std::cout << "L_aug = \n" << L_aug << "\n";

    // create augmented sigma points
    Xsig_aug.fill(0.0);
    Xsig_aug.col(0) = x_aug;
    for (long i = 0; i < n_aug_; ++i)
    {
        Xsig_aug.col(i+1) = x_aug + (std::sqrt(lambda_ + n_aug_) * L_aug.col(i));
        Xsig_aug.col(i+n_aug_+1) = x_aug - (std::sqrt(lambda_ + n_aug_) * L_aug.col(i));
    }

    // propagate the sigma points through the prediction model
    double dt2 = delta_t * delta_t;

    for (long i = 0; i < 2 * n_aug_ + 1; ++i)
    {
        double px   { Xsig_aug(0, i) };
        double py   { Xsig_aug(1, i) };
        double v    { Xsig_aug(2, i) };
        double psi  { Xsig_aug(3, i) };
        double dpsi { Xsig_aug(4, i) };

        double nu_a { Xsig_aug(5, i) };
        double nu_ddpsi { Xsig_aug(6, i) };

        // avoid division by zero
        if (std::abs(dpsi) < 0.001)
        {
            px += (v * delta_t * std::cos(psi)) + (0.5 * dt2 * nu_a * std::cos(psi));
            py += (v * delta_t * std::sin(psi)) + (0.5 * dt2 * nu_a * std::sin(psi));
        }
        else
        {
            px += ((v/dpsi) * ( std::sin(psi + (dpsi * delta_t)) - std::sin(psi))) + (0.5 * dt2 * nu_a * std::cos(psi));
            py += ((v/dpsi) * (-std::cos(psi + (dpsi * delta_t)) + std::cos(psi))) + (0.5 * dt2 * nu_a * std::sin(psi));
        }
        
        v += (delta_t * nu_a);
        psi += (delta_t * dpsi) + (0.5 * dt2 * nu_ddpsi);
        dpsi += (delta_t * nu_ddpsi);

        // write predicted sigma points into right column
        Xsig_pred_(0, i) = px;
        Xsig_pred_(1, i) = py;
        Xsig_pred_(2, i) = v;
        Xsig_pred_(3, i) = psi;
        Xsig_pred_(4, i) = dpsi;
    }

    //create vector for predicted state
    VectorXd x_pred = VectorXd(n_x_);
    
    //create covariance matrix for prediction
    MatrixXd P_pred = MatrixXd(n_x_, n_x_);
    
    x_pred.fill(0.0);
    P_pred.fill(0.0);

    // predict state mean
    for (long i = 0; i < Xsig_pred_.cols(); ++i)
    {
        x_pred += weights_(i) * Xsig_pred_.col(i);
    }

    // wrap the angle between [-180, 180]
    normalizeAngle(x_(3));

    // predict state covariance matrix
    for (long i = 0; i < Xsig_pred_.cols(); i++)
    {
        VectorXd xDiff = Xsig_pred_.col(i) - x_pred;

        normalizeAngle(xDiff(3));

        P_pred += weights_(i) * xDiff * xDiff.transpose();
    }

    x_ = x_pred;
    P_ = P_pred;
}

void UKF::UpdateLidar(MeasurementPackage meas_package)
{
    /**
     * TODO: Complete this function! Use lidar data to update the belief 
     * about the object's position. Modify the state vector, x_, and 
     * covariance, P_.
     * You can also calculate the lidar NIS, if desired.
     */

    const long n_z = meas_package.raw_measurements_.size();

    Eigen::VectorXd z = Eigen::VectorXd(n_z);
    z(0) = meas_package.raw_measurements_(0);
    z(1) = meas_package.raw_measurements_(1);

    // measurement covariance
    Eigen::MatrixXd R = Eigen::MatrixXd(n_z, n_z);
    R << std_laspx_*std_laspx_,      0,
                             0, std_laspy_*std_laspy_;

    // measurement matrix
    Eigen::MatrixXd H = Eigen::MatrixXd(n_z, n_x_);
    H << 1, 0, 0, 0, 0,
         0, 1, 0, 0, 0;

    Eigen::VectorXd z_pred = H * x_;
    Eigen::VectorXd y = z - z_pred;
    Eigen::MatrixXd Ht = H.transpose();
    Eigen::MatrixXd S = H * P_ * Ht + R;
    Eigen::MatrixXd Si = S.inverse();
    Eigen::MatrixXd PHt = P_ * Ht;
    Eigen::MatrixXd K = PHt * Si;

    //new estimate
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_x_, n_x_);

    x_ += K * y;    
    P_ = (I - K * H) * P_;
}

void UKF::UpdateRadar(MeasurementPackage meas_package)
{
    /**
     * TODO: Complete this function! Use radar data to update the belief 
     * about the object's position. Modify the state vector, x_, and 
     * covariance, P_.
     * You can also calculate the radar NIS, if desired.
     */

    const long n_z = meas_package.raw_measurements_.size();

    // create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

    // mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);

    // measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);

    Zsig.fill(0.0);
    // transform sigma points into measurement space
    for (long i = 0; i < Xsig_pred_.cols(); ++i)
    {
        const double px   = Xsig_pred_(0, i);
        const double py   = Xsig_pred_(1, i);
        const double v    = Xsig_pred_(2, i);
        const double psi  = Xsig_pred_(3, i);
        //const double dpsi = Xsig_pred_(4, i);

        double rho  = std::sqrt( px*px + py*py );
        double phi  = std::atan2( py , px );
        double drho = ((v * std::cos(psi) * px) + (v * std::sin(psi) * py)) / rho;

        Zsig(0, i) = rho;
        Zsig(1, i) = phi;
        Zsig(2, i) = drho;
    }

    // calculate mean predicted measurement
    for (long i = 0; i < Zsig.cols(); ++i)
    {
        z_pred += weights_(i) * Zsig.col(i);
    }

    // calculate innovation covariance matrix S
    for (long i = 0; i < Zsig.cols(); i++)
    {
        VectorXd zDiff = Zsig.col(i) - z_pred;

        // wrap the bearing angle [-180, 180]
        normalizeAngle(zDiff(1));

        S += weights_(i) * zDiff * zDiff.transpose();
    }

    MatrixXd R(n_z, n_z);
    R << std_radr_*std_radr_,                       0,                     0,
                           0, std_radphi_*std_radphi_,                     0,
                           0,                       0, std_radrd_*std_radrd_;

    S += R;

    // calculate cross correlation matrix

    // create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    Tc.fill(0.0);

    for (long i = 0; i < weights_.rows(); i++)
    {
        VectorXd xDiff = Xsig_pred_.col(i) - x_;
        VectorXd zDiff = Zsig.col(i) - z_pred;

        normalizeAngle(xDiff(3));
        normalizeAngle(zDiff(1));

        Tc += weights_(i) * xDiff * zDiff.transpose();
    }
    
    // calculate Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    // update state mean and covariance matrix
    Eigen::VectorXd z = Eigen::VectorXd(n_z);
    z(0) = meas_package.raw_measurements_[0];
    z(1) = meas_package.raw_measurements_[1];
    z(2) = meas_package.raw_measurements_[2];

    VectorXd z_residual = z - z_pred;
    normalizeAngle(z_residual(1));

    x_ += K * z_residual;
    P_ -= K * S * K.transpose();

    normalizeAngle(x_(3));
}
