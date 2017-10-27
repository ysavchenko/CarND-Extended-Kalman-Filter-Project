#include "kalman_filter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

#ifndef M_PI
#define M_PI (3.141592653589793)
#endif

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {
  Q_ = MatrixXd::Zero(4, 4);
  Hj_ = MatrixXd::Zero(4, 3);
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &R_EKF_in) {
  x_ = x_in;

  long x_size = x_.size();
	I_ = MatrixXd::Identity(x_size, x_size);

  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  R_EKF_ = R_EKF_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;

  VectorXd y = z - z_pred;
  UpdateCommon(y, H_, R_);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Calculate h(x)
  float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);

  if (px < 0.001) {
    cout << "px near 0, cannot update" << endl;
  }

  VectorXd hx = VectorXd(3);
  float rho = sqrt(px * px + py * py);
  float theta = atan2(py, px); // atan2 already returns value in -pi to pi range, no need to normalize

  if (rho < 0.001) {
    cout << "rho near 0, cannot update" << endl;
  }

  float rho_dot = (px * vx + py * vy) / rho;
  hx << rho, theta, rho_dot;

  VectorXd y = z - hx;

  // Normalize theta angles difference
  while (y(1) > M_PI) y(1) = y(1) - 2 * M_PI;
  while (y(1) < -M_PI) y(1) = y(1) + 2 * M_PI;

  UpdateCommon(y, Hj_, R_EKF_);
}

// Common part of both lidar and radar update functions
void KalmanFilter::UpdateCommon(const VectorXd &y, const MatrixXd &H, const MatrixXd &R) {
  MatrixXd Ht = H.transpose();

  MatrixXd S = H * P_ * Ht + R;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	P_ = (I_ - K * H) * P_;
}
