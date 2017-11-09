#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;
  
  previous_timestamp_ = 0;
  
  // initializing matrices
  Hj_ = MatrixXd(3, 4);
  
  
  // measurement function laser
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
  0, 1, 0, 0;
  
  
  //measurement covariance matrix - laser
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
  0, 0.0225;
  
  //measurement covariance matrix - radar
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
    0, 0.0009, 0,
    0, 0, 0.09;
  
  VectorXd x_ = VectorXd(4);
  x_.setZero();
  
  MatrixXd P_=Eigen::MatrixXd(4,4);
  P_<<1000,0,0,0,
    0,1000,0,0,
    0,0,1000,0,
    0,0,0,1000;
  
  MatrixXd F_=Eigen::MatrixXd(4,4);
  F_<<1,0,0,0,
    0,1,0,0,
    0,0,1,0,
    0,0,0,1;
  
  
  //process covariance matrix acceleration noise
  noise_ax = 9;
  noise_ay = 9;
  
  MatrixXd Q_=Eigen::MatrixXd(4,4);
  Q_.setZero();
  
  ekf_.Init(x_, P_, F_, H_laser_, R_laser_, R_radar_, Q_);
  
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  
  const bool use_radar=1;
  const bool use_lidar=1;
  
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_.setZero();
    
    
    if ((measurement_pack.sensor_type_ == MeasurementPackage::RADAR)&&use_radar) {
      /**
       Convert radar from polar to cartesian coordinates and initialize state.
       */
      float r=measurement_pack.raw_measurements_[0];
      float phi=measurement_pack.raw_measurements_[1];
      
      ekf_.x_[0]=r*cos(phi);
      ekf_.x_[1]=r*sin(phi);
      previous_timestamp_=measurement_pack.timestamp_;
      
      is_initialized_ = true;
      
    }
    else if ((measurement_pack.sensor_type_ == MeasurementPackage::LASER)&&use_lidar) {
      /**
       Initialize state.
       */
      ekf_.x_[0]=measurement_pack.raw_measurements_[0];
      ekf_.x_[1]=measurement_pack.raw_measurements_[1];
      previous_timestamp_=measurement_pack.timestamp_;
      
      is_initialized_ = true;
      
    }
    
    // done initializing, no need to predict or update
    return;
  }
  
  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  
  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;  //dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  
  //Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  
  //set the process covariance matrix Q
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
  0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
  dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
  0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
  
  
  ekf_.Predict();
  
  cout<<"PREDICT \n";
  cout << ekf_.x_ << endl;
  cout << ekf_.P_ << endl;
  
  /*****************************************************************************
   *  Update
   ****************************************************************************/
  
  
  if ((measurement_pack.sensor_type_ == MeasurementPackage::RADAR)&&use_radar) {
    // Radar updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else if((measurement_pack.sensor_type_==MeasurementPackage::LASER)&& use_lidar){
    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);
  }
  
  // print the output
  cout<<"UPDATE ";
  cout << measurement_pack.sensor_type_ << endl;
  cout << ekf_.x_ << endl;
  cout << ekf_.P_ << endl;
  cout << endl;
}
