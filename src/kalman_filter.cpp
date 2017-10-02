#include "kalman_filter.h"
#include <math.h>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  /**
  TODO:
    * predict the state
  */
  // From quiz, Lesson 5, section 12
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::UpdateXandP(const VectorXd &z, VectorXd &y) {
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  //new estimate
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  // From quiz, Lesson 5, section 12
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  UpdateXandP(z, y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  // From quiz, Lesson 5, section 14
  float px = this->x_[0];
  float py = this->x_[1];
  float vx = this->x_[2];
  float vy = this->x_[3];
  float rho = sqrt(px * px + py * py);
  if (py == 0 and px == 0) {
    return;
  }
  if (rho < 0.0001) {
    return;
  }
  float theta = atan2(py, px);
  float ro_dot = (px * vx + py * vy) / rho;
  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, ro_dot;
  VectorXd y = z - z_pred;
  // Tips and Tricks - Normalizing Angles: adjust phi to between -pi and pi
  y[1] = atan2(sin(y[1]), cos(y[1]));

  UpdateXandP(z, y);
}
