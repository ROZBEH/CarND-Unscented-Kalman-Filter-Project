#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <fstream>
#include <vector>
#include "graph.h"
using namespace std;


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
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  x_.setZero();

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_.setZero();

  // Process noise standard deviation longitudinal acceleration in m/s^2
  // 0 to 20 km/h in 10 second
  std_a_ = 1.4;

  // Process noise standard deviation yaw acceleration in rad/s^2
  //0 to 2 rad/s in 10 second
  std_yawdd_ = 0.3;
  
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
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  // at the beginning nothing is initialized
  is_initialized_ = false;

  n_x_ = 5;
  counter = 0;

  n_aug_ = n_x_ + 2;

  lambda_ = 3 - n_aug_;

  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  time_us_ = 0.0;

  weights_ = VectorXd(2*n_aug_+1);
  R_lidar = MatrixXd(2,2);
  R_lidar << std_laspx_*std_laspx_,0,
              0,std_laspy_*std_laspy_;
  R_radar = MatrixXd(3,3);
  R_radar << std_radr_*std_radr_, 0, 0,
              0, std_radphi_*std_radphi_, 0,
              0, 0,std_radrd_*std_radrd_;
  // setting values of the weights
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  double weight = 0.5/(n_aug_+lambda_);
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
    weights_(i) = weight;
  }
  // P_ << MatrixXd::Identity(5,5);
  // first I initialized x_ with zeros. Once I get the first measurement I will initialize it with something better!
  x_ << 0, 0, 0, 1, 0.1;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (!is_initialized_) {
    // z = VectorXd(3);
    // first measurement
    P_ << 0.15,    0, 0, 0, 0,
               0, 0.15, 0, 0, 0,
               0,    0, 1, 0, 0,
               0,    0, 0, 1, 0,
               0,    0, 0, 0, 1;
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      // Rouzbeh: Changing the radar coordinates into cartesian coordinates
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
      double rho_dot = meas_package.raw_measurements_[2];
      double x = rho * cos(phi);
      if ( x < 0.0001 ) {
        x = 0.0001;
      }
      double y = rho * sin(phi);
      if ( y < 0.0001 ) {
        y = 0.0001;
      }
      double vx = rho_dot * cos(phi);
      double vy = rho_dot * sin(phi);
      double v = sqrt(vx*vx + vy*vy);
      x_(0) = x ;
      x_(1) = y ;
      x_(2) = v ;
      // z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], meas_package.raw_measurements_[2];
    }
    else {
      /**
      Initialize state.
      */
      x_(0) = meas_package.raw_measurements_[0];
      x_(1) = meas_package.raw_measurements_[1];
      // z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0;
    }
    if (fabs(x_(0)) < 0.001 and fabs(x_(1)) < 0.001){
    x_(0) = 0.001;
    x_(1) = 0.001;
    }
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;

    return;
  }
  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0; //dt - expressed in seconds
  time_us_ = meas_package.timestamp_;

  // Prediction step
  Prediction(delta_t);
  //Update if this is radar or lidar
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    UpdateRadar(meas_package);
  }
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_){
    UpdateLidar(meas_package);
  }

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  VectorXd x_aug = VectorXd(7);
  x_aug.setZero();
  MatrixXd P_aug = MatrixXd(7, 7);
  P_aug.setZero();
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1); 
  Xsig_aug.setZero();
  x_aug << x_, 0, 0;
  P_aug.topLeftCorner(5, 5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  MatrixXd A = P_aug.llt().matrixL();
  double lambda_sq = sqrt(lambda_ + n_aug_);
  Xsig_aug.col(0) = x_aug;
  MatrixXd temp = lambda_sq*A;
  Xsig_aug.block(0, 1, 7, 7) = (temp).colwise() + x_aug;
  Xsig_aug.block(0, 8, 7, 7) = (-temp).colwise() + x_aug;

  double delta_t_2 = delta_t * delta_t;
    for (int i = 0; i< 2*n_aug_+1; i++)
  {
      if (Xsig_aug(4,i) > 0.001){
      Xsig_pred_(0,i) = Xsig_aug(0,i) + (Xsig_aug(2,i)/Xsig_aug(4,i))*(sin(Xsig_aug(3,i)+Xsig_aug(4,i)*delta_t)-sin(Xsig_aug(3,i))) + 0.5 *(delta_t_2)*cos(Xsig_aug(3,i))*Xsig_aug(5,i);
      Xsig_pred_(1,i) = Xsig_aug(1,i) + (Xsig_aug(2,i)/Xsig_aug(4,i))*(-cos(Xsig_aug(3,i)+Xsig_aug(4,i)*delta_t)+cos(Xsig_aug(3,i))) + 0.5 *(delta_t_2)*sin(Xsig_aug(3,i))*Xsig_aug(5,i);
      Xsig_pred_(2,i) = Xsig_aug(2,i) + 0 + delta_t * Xsig_aug(5,i) ;
      Xsig_pred_(3,i) = Xsig_aug(3,i) + Xsig_aug(4,i) * delta_t + 0.5 * delta_t_2 * Xsig_aug(6,i) ;
      Xsig_pred_(4,i) = Xsig_aug(4,i) + 0 + delta_t * Xsig_aug(6,i); 
      }
      else{
      Xsig_pred_(0,i) = Xsig_aug(0,i) + Xsig_aug(2,i) * cos(Xsig_aug(3,i)) * delta_t + 0.5 *(delta_t_2)*cos(Xsig_aug(3,i))*Xsig_aug(5,i);;
      Xsig_pred_(1,i) = Xsig_aug(1,i) + Xsig_aug(2,i) * sin(Xsig_aug(3,i)) * delta_t + 0.5 *(delta_t_2)*sin(Xsig_aug(3,i))*Xsig_aug(5,i);
      Xsig_pred_(2,i) = Xsig_aug(2,i) + 0 + delta_t * Xsig_aug(5,i) ;
      Xsig_pred_(3,i) = Xsig_aug(3,i) + Xsig_aug(4,i) * delta_t + 0.5 * delta_t_2 * Xsig_aug(6,i) ;
      Xsig_pred_(4,i) = Xsig_aug(4,i) + 0 + delta_t * Xsig_aug(6,i); 
      }
  }
  x_ = Xsig_pred_ * weights_;
  //predict state covariance matrix
  P_.fill(0.0);
  MatrixXd temp1 = Xsig_pred_.colwise() - x_;
  for (int j = 0; j < 2 * n_aug_ + 1; j++){
    while (temp1(3,j)> M_PI) temp1(3,j)-=2.*M_PI;
    while (temp1(3,j)<-M_PI) temp1(3,j)+=2.*M_PI;
  }
  //replicate the weights
  MatrixXd replic_weight = weights_.replicate(1,5);
  P_ = P_ + (temp1)*(temp1.transpose().cwiseProduct(replic_weight));
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  int n_z = 2;
  VectorXd z = VectorXd(n_z);
  z.setZero();
  z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  Zsig.setZero();
  VectorXd z_pred = VectorXd(n_z);
  z_pred.setZero();
  // z_pred.fill(0.0);
  MatrixXd S = MatrixXd(n_z,n_z);
  S.setZero();
  for (int j = 0 ; j < 2 * n_aug_ + 1; j++){
      Zsig(0,j) = Xsig_pred_(0,j); 
      Zsig(1,j) = Xsig_pred_(1,j);
    }
  //calculate mean predicted measurement
  z_pred = Zsig * weights_;
  //calculate innovation covariance matrix S
  
  MatrixXd temp = Zsig.colwise() - z_pred;
  for (int j = 0; j < 2 * n_aug_ + 1; j++){
      while (temp(1,j)> M_PI) temp(1,j)-=2.*M_PI;
      while (temp(1,j)<-M_PI) temp(1,j)+=2.*M_PI;
    }
  //replicate the weights
  MatrixXd replic_weight = weights_.replicate(1,n_z);
  S = (temp)*(temp.transpose().cwiseProduct(replic_weight));
  S = S + R_lidar;
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.setZero();
  MatrixXd temp1 = Xsig_pred_.colwise() - x_;
  for (int j = 0; j < 2 * n_aug_ + 1; j++){
      while (temp1(1,j)> M_PI) temp1(1,j)-=2.*M_PI;
      while (temp1(1,j)<-M_PI) temp1(1,j)+=2.*M_PI;
    }
  //replicate the weights
  Tc = (temp1)*(temp.transpose().cwiseProduct(replic_weight));
  //calculate Kalman gain K;
  MatrixXd K = MatrixXd(n_x_, n_z);
  K.setZero();
  K = Tc*S.inverse();
  //update state mean and covariance matrix
  x_ = x_ + K * (z - z_pred);
  P_ = P_ - K * S * K.transpose();
  float nis_temp = (z - z_pred).transpose() * S.inverse() * (z - z_pred);
  NIS_lidar.push_back(nis_temp);
  counter += 1;
  if (counter == 498){
    if (counter == 498 || counter == 497){
    // ofstream outFile("my_file_lidar.txt");
    // // the important part
    // for (const auto &e : NIS_lidar) outFile << e << "\n";

  plot p;
  for(int a=0;a<100;a++) {
    vector<float> x,y;
    for(int k=a;k<a+200;k++) {
      x.push_back(k);
      y.push_back(k*k);
    }
    p.plot_data(x,y);
  }
  }
  }
  // cout<< "NIS_lidar = "<<NIS_lidar<<" -------";
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  int n_z = 3;
  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], meas_package.raw_measurements_[2];
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  VectorXd z_pred = VectorXd(n_z);
  MatrixXd S = MatrixXd(n_z,n_z);
  // z_pred.fill(0.0);
  S.setZero();
  for (int j = 0 ; j < 2 * n_aug_ + 1; j++){
      double pxpy = sqrt(Xsig_pred_(0,j)*Xsig_pred_(0,j)+Xsig_pred_(1,j)*Xsig_pred_(1,j));
      Zsig(0,j) = pxpy; 
      Zsig(1,j) = atan2(Xsig_pred_(1,j),Xsig_pred_(0,j));
      Zsig(2,j) = Xsig_pred_(2,j)*(Xsig_pred_(0,j)*cos(Xsig_pred_(3,j))+Xsig_pred_(1,j)*sin(Xsig_pred_(3,j)))/pxpy;
    }
  //calculate mean predicted measurement
  z_pred = Zsig * weights_;
  //calculate innovation covariance matrix S
  
  MatrixXd temp = Zsig.colwise() - z_pred;
  for (int j = 0; j < 2 * n_aug_ + 1; j++){
      while (temp(1,j)> M_PI) temp(1,j)-=2.*M_PI;
      while (temp(1,j)<-M_PI) temp(1,j)+=2.*M_PI;
    }
  //replicate the weights
  MatrixXd replic_weight = weights_.replicate(1,n_z);
//   temp size= 3 15
//  replic_weight size= 15 3
//  S size = 3 3
  S = (temp)*(temp.transpose().cwiseProduct(replic_weight));
  S = S + R_radar;
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.setZero();
  MatrixXd temp1 = Xsig_pred_.colwise() - x_;
  for (int j = 0; j < 2 * n_aug_ + 1; j++){
      while (temp1(1,j)> M_PI) temp1(1,j)-=2.*M_PI;
      while (temp1(1,j)<-M_PI) temp1(1,j)+=2.*M_PI;
    }

  //replicate the weights
  Tc =  (temp1)*(temp.transpose().cwiseProduct(replic_weight));
  //calculate Kalman gain K;
  MatrixXd K = MatrixXd(n_x_, n_z);
  K.setZero();
  K = Tc*S.inverse();
  //update state mean and covariance matrix
  x_ = x_ + K * (z - z_pred);
  P_ = P_ - K * S * K.transpose();
  float nis_temp = (z - z_pred).transpose() * S.inverse() * (z - z_pred);
  NIS_radar.push_back(nis_temp);
  counter += 1;
  if (counter == 498 || counter == 497){
    // ofstream outFile("my_file_radar.txt");
    // // the important part
    // for (const auto &e : NIS_radar) outFile << e << "\n";
  plot p;
  for(int a=0;a<100;a++) {
    vector<float> x,y;
    for(int k=a;k<a+200;k++) {
      x.push_back(k);
      y.push_back(k*k);
    }
    p.plot_data(x,y);
  }
  }
  // cout<< "NIS_radar = "<<NIS_radar<<endl;
}
