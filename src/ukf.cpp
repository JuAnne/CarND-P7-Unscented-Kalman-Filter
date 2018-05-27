#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true; //false;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true; //false;

  // State dimension
  n_x_ = 5;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);  // 5 x 5

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.2; // tuned
  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5; // tuned
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // time when the state is true, in us
  time_us_ = 0;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // initial predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_+ 1);  // 5 x 15

  // Weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);   //15 x 1

  // initially set NIS
  NIS_lidar_ = 0;
  NIS_radar_ = 0;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {

    //Initialize the state ukf_.x_ with the first measurement.
    //Create the covariance matrix.

    // first measurement
    cout << "UKF: " << endl;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {   
      //Convert radar from polar to cartesian coordinates.
      double ro     = meas_package.raw_measurements_(0);
      double theta    = meas_package.raw_measurements_(1);
      double ro_dot = meas_package.raw_measurements_(2);

      // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
      x_(0) = ro * cos(theta);
      x_(1) = ro * sin(theta);
      x_(2) = 4 ; // tuned for velocity
      x_(3) = ro_dot * cos(theta);
      x_(4) = ro_dot * sin(theta);

      // state covariance matrix (need tuning)
      P_ << std_radr_*std_radr_,    0,    0,    0,    0,
               0,    std_radr_*std_radr_,    0,    0,    0,
               0,    0,    1,    0,    0,
               0,    0,    0,    std_radphi_,    0,
               0,    0,    0,    0,    std_radrd_;

    }

    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      //Initialize px and py.
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);
      x_(2) = 4; // tuned
	x_(3) = 0.5; // tuned
	x_(4) = 0; // tuned

      // state covariance matrix (need tuning)
      P_ << std_laspx_*std_laspx_,    0,    0,    0,    0,
               0,    std_laspy_*std_laspy_,    0,    0,    0,
               0,    0,    1,    0,    0,
               0,    0,    0,    1,    0,
               0,    0,    0,    0,    1;

    }

    time_us_ = meas_package.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  //compute the time elapsed (in seconds) between the current and previous measurements
  long long dt_ll = (meas_package.timestamp_ - time_us_);
  double delta_t = (double)dt_ll/ 1000000.0;	//delta_t - expressed change in time in seconds
  time_us_ = meas_package.timestamp_;

  //Predicts sigma points, the state, and the state covariance matrix.
  Prediction(delta_t);

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  // Use the sensor type to perform the update step.
  // Update the state and covariance matrices.
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR
  	&& use_radar_ == true) {
    // Radar updates
    UpdateRadar(meas_package);
  }

  if (meas_package.sensor_type_ == MeasurementPackage::LASER
  	&& use_laser_ == true)  {
    // Laser updates
    UpdateLidar(meas_package);
  }

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  //create augmented mean vector
  VectorXd x_aug_ = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug_ = MatrixXd(n_aug_, n_aug_);  // 7 x 7

  //augmented mean state
  x_aug_.head(5) = x_;
  x_aug_(5) = 0;
  x_aug_(6) = 0;

  //augmented covariance matrix
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(5,5) = P_;
  P_aug_(5,5) = std_a_*std_a_; // Linear Acceleration Noise m^2/s^4
  P_aug_(6,6) = std_yawdd_*std_yawdd_; // Yaw Acceleration Noise rad^2/s^4

  //create square root matrix
  MatrixXd L_ = P_aug_.llt().matrixL();  // 7 x 7

  //create augmented sigma points
  MatrixXd Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1); // 7 x 15
  Xsig_aug_.col(0)  = x_aug_;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug_.col(i+1)       = x_aug_ + sqrt(lambda_+n_aug_) * L_.col(i);
    Xsig_aug_.col(i+1+n_aug_) = x_aug_ - sqrt(lambda_+n_aug_) * L_.col(i);
  }

  //std::cout << "Xsig_aug_ = " << std::endl << Xsig_aug_ << std::endl;

  /*****************************************************************************
   *  Predict Sigma Points
   ****************************************************************************/
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug_(0,i);
    double p_y = Xsig_aug_(1,i);
    double v = Xsig_aug_(2,i);
    double yaw = Xsig_aug_(3,i);
    double yawd = Xsig_aug_(4,i);
    double nu_a = Xsig_aug_(5,i);
    double nu_yawdd = Xsig_aug_(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  /*****************************************************************************
   *  Predict the state Mean and Covariance
   ****************************************************************************/
  // set weights
  double weight_0 = lambda_/(lambda_+n_aug_); // lambda / 3
  weights_(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) { 
    double weight = 0.5/(n_aug_+lambda_); // 0.5 / 3
    weights_(i) = weight;
  }

  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_+ weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }


}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /*****************************************************************************
   *  Predict Lidar Measurement
   ****************************************************************************/
  //set measurement dimension, lidar can measure px and py
  int n_z_ = 2;
  //create matrix for sigma points in measurement space
  MatrixXd Zsig_ = MatrixXd(n_z_, 2 * n_aug_ + 1);    // 2 x 15

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) { 

    // extract values for better readability
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);

    // measurement model
    Zsig_(0,i) = p_x;
    Zsig_(1,i) = p_y;
  }

  //mean predicted measurement
  VectorXd z_pred_ = VectorXd(n_z_);   // 2 x 1
  z_pred_.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred_ = z_pred_ + weights_(i) * Zsig_.col(i);
  }

  //calculate innovation covariance matrix S
  MatrixXd S_ = MatrixXd(n_z_,n_z_);   // 2 x 2
  S_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) { 
    //residual
    VectorXd z_diff = Zsig_.col(i) - z_pred_;  // 2 x 1

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S_ = S_ + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R_ = MatrixXd(n_z_,n_z_);    // 2 x 2
  R_ <<    std_laspx_*std_laspx_, 0,
          0, std_laspy_*std_laspy_;
  S_ = S_ + R_;  // 2 x 2

  /*****************************************************************************
   *  Calculate cross correlation matrix
   ****************************************************************************/
  //create vector for incoming lidar measurement
  VectorXd z_ = VectorXd(n_z_);  // 2 x 1  
  z_(0) = meas_package.raw_measurements_(0);
  z_(1) = meas_package.raw_measurements_(1);

  //create matrix for cross correlation Tc
  MatrixXd Tc_ = MatrixXd(n_x_, n_z_);   // 5 x 2

  //calculate cross correlation matrix
  Tc_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) { 

    //residual
    VectorXd z_diff = Zsig_.col(i) - z_pred_;  // 2 x 1
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;   // 5 x 1
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc_ = Tc_ + weights_(i) * x_diff * z_diff.transpose();   // 5 x 2
  }

  /*****************************************************************************
   *  Calculate Kalman gain
   ****************************************************************************/
  //Kalman gain K_;
  MatrixXd K_ = Tc_ * S_.inverse();   // 5 x 2

  //residual
  VectorXd z_diff = z_ - z_pred_;   // 2 x 1

  /*****************************************************************************
   *  Calculate Lidar NIS(Normalized Innovation Squared)
   ****************************************************************************/
  NIS_lidar_ = z_diff.transpose() * S_.inverse() * z_diff;
  //std::cout << "NIS_lidar_ = " << NIS_lidar_ << std::endl;

  /*****************************************************************************
   *  Update Radar State Mean and Covariance
   ****************************************************************************/
  x_ = x_ + K_ * z_diff;   // 5 x 1
  P_ = P_ - K_*S_*K_.transpose();  // 5 x 5

}


  
/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /*****************************************************************************
   *  Predict Radar Measurement
   ****************************************************************************/
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z_ = 3;
  //create matrix for sigma points in measurement space
  MatrixXd Zsig_ = MatrixXd(n_z_, 2 * n_aug_ + 1);    // 3 x 15

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) { 

    // extract values for better readability
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig_(0,i) = sqrt(p_x*p_x + p_y*p_y);  //r
    Zsig_(1,i) = atan2(p_y,p_x);  //phi
    Zsig_(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y); //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred_ = VectorXd(n_z_);   // 3 x 1
  z_pred_.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred_ = z_pred_ + weights_(i) * Zsig_.col(i);
  }

  //calculate innovation covariance matrix S
  MatrixXd S_ = MatrixXd(n_z_,n_z_);   // 3 x 3
  S_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) { 
    //residual
    VectorXd z_diff = Zsig_.col(i) - z_pred_;  // 3 x 1

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S_ = S_ + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R_ = MatrixXd(n_z_,n_z_);    // 3 x 3
  R_ <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
  S_ = S_ + R_;  // 3 x 3

  /*****************************************************************************
   *  Calculate cross correlation matrix
   ****************************************************************************/
  //create vector for incoming radar measurement
  VectorXd z_ = VectorXd(n_z_);  // 3 x 1  
  z_(0) = meas_package.raw_measurements_(0);
  z_(1) = meas_package.raw_measurements_(1);
  z_(2) = meas_package.raw_measurements_(2);

  //create matrix for cross correlation Tc
  MatrixXd Tc_ = MatrixXd(n_x_, n_z_);   // 5 x 3

  //calculate cross correlation matrix
  Tc_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) { 

    //residual
    VectorXd z_diff = Zsig_.col(i) - z_pred_;  // 3 x 1
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;   // 5 x 1
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc_ = Tc_ + weights_(i) * x_diff * z_diff.transpose();   // 5 x 3
  }

  /*****************************************************************************
   *  Calculate Kalman gain
   ****************************************************************************/
  //Kalman gain K_;
  MatrixXd K_ = Tc_ * S_.inverse();   // 5 x 3

  //residual
  VectorXd z_diff = z_ - z_pred_;   // 3 x 1

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  /*****************************************************************************
   *  Calculate Radar NIS(Normalized Innovation Squared)
   ****************************************************************************/
  NIS_radar_ = z_diff.transpose() * S_.inverse() * z_diff;
  //std::cout << "NIS_radar_ = " << NIS_radar_ << std::endl;

  /*****************************************************************************
   *  Update Radar State Mean and Covariance
   ****************************************************************************/
  x_ = x_ + K_ * z_diff;   // 5 x 1
  P_ = P_ - K_*S_*K_.transpose();  // 5 x 5

}
