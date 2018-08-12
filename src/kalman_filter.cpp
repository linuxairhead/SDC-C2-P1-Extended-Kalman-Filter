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
  /**
    * predict the state
  */
  x_ = F_ * x_; // u = 0
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  x_ = x_ + K * y;
  P_ = (I_ - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * update the state by using Extended Kalman Filter equations
  */
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  // If ro == 0, skip the update step to avoid dividing by zero.
  if( px == 0. && py == 0. )
    return;

  float rho = sqrt( px*px + py*py );

  VectorXd z_pred(3);
  z_pred << rho, atan2( py, px ), ( px*vx + py*vy )/rho;

  // Update the state using Extended Kalman Filter equations
  VectorXd y = z - z_pred;
  if( y[1] > PI )
    y[1] -= 2.f*PI;
  if( y[1] < -PI )
    y[1] += 2.f*PI;

  MatrixXd Hj_ = tools.CalculateJacobian( x_ );
  MatrixXd S = Hj_*P_* Hj_.transpose() + R_;
  MatrixXd K =  P_*Hj_.transpose()*S.inverse();

  // Compute new state
  x_ = x_ + ( K * y );
  P_ = ( I_ - K * Hj_ ) * P_;
}
