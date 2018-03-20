#include <iostream>
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;  // x' = F x + u 
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;  // P' = F P F^T + Q
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;  // y = z − Hx'
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;  // S = H P' H^T + R
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;  // K = P' H^T S^−1

  //new estimate
  x_ = x_ + (K * y);  // x = x' + Ky
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;  // P = (I − KH)P'
}

double NormalizeAngleInRadians(const double angle) {
  double const pi = 3.141592653589793238462643383279502884;
  double const two_pi = 2 * pi;
  double new_angle = angle;
  while (angle > two_pi) {
    new_angle -= two_pi;
  }
  return new_angle;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  double rho = sqrt(pow(x_(0), 2) + pow(x_(1), 2));
  double theta = atan2(x_(1), x_(0));
  if (rho != 0) {
    double rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
    VectorXd z_pred = VectorXd::Zero(3);
    z_pred << rho, theta, rho_dot;
    VectorXd y = z - z_pred;  // y = z − h(x')
    y(1) = NormalizeAngleInRadians(y(1));
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;  // S = H P' H^T + R
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;  // K = P' H^T S^−1
  } else {
    std::cout << "Error with division by zero when doing the Extended Kalman filter update." << std::endl;
  }
}
