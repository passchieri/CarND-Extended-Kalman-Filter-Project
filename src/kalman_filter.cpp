#include "kalman_filter.h"
#include "iostream"
#define _USE_MATH_DEFINES
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &R_radar_in,
                        MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in; //from the laser scanner state -> measurement state
  R_ = R_in; //from the laser scanner measurement noise matris
  R_radar=R_radar_in;
  Q_ = Q_in;
  
  cout << x_ << "x" << endl;

  cout << P_ << "P" << endl;

  cout << F_ << "F" << endl;

  cout << H_ << "H" << endl;

  cout << R_ << "R" << endl;
}

void KalmanFilter::Predict() {
  x_=F_*x_;
  P_=F_*P_*F_.transpose() + Q_;
  
}

void KalmanFilter::Update(const VectorXd &z) { //only to be used for the laser scanner, because H_ and R_ that are used are from the laser scanner
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  //MatrixXd Si = S.inverse();
  //MatrixXd PHt = P_ * Ht;
  MatrixXd K = P_ * Ht * S.inverse();
  
  
//  cout << z << "z" <<endl;
//  cout << z_pred << "pred" << endl;
//  cout << x_ << " x before" << endl;
  //new estimate
 x_ = x_ + (K * y);
//  cout << x_ << " x after" << endl;

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  Eigen::MatrixXd Hj=tools.CalculateJacobian(x_);
  
  VectorXd z_pred = tools.RadarH(x_);
  VectorXd y = z - z_pred;
  while (y[1] >=M_PI) {
    y[1]-=2*M_PI;
  }
  while (y[1] <-M_PI) {
    y[1]+=2*M_PI;
  }
  MatrixXd Ht = Hj.transpose();
  MatrixXd S = Hj * P_ * Ht + R_radar;

  MatrixXd K = P_ * Ht * S.inverse();

  //new estimate
  x_ = x_ + (K * y);  
}
