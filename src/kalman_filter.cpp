#include "kalman_filter.h"
#include <string>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {
  /*
   * private variable for debug
   */
  fn = "Constructor";
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {

  fn = "Init";
  KF_DEBUG(fn, "Start");  

  x_ = x_in;
  I_ =  MatrixXd::Identity(x_.size(), x_.size());
  P_ =  I_;
  P_(2,2) = 1000;
  P_(3,3) = 1000;
  F_ = I_;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
  
  KF_DEBUG(fn, "End");
}

void KalmanFilter::Predict() {
  
  fn = "Predict";
  KF_DEBUG(fn, "start");

  /**
    * predict the state
  */
  KF_DEBUG(fn, "x_");
  KF_DEBUG(fn, x_ );
  KF_DEBUG(fn, "F_");
  KF_DEBUG(fn, F_ );
  x_ = F_ * x_; // u = 0
  P_ = F_ * P_ * F_.transpose() + Q_;
  KF_DEBUG(fn, x_ );
}

void KalmanFilter::Update(const VectorXd &z) {

  fn = "KalmanFilterUpdate";
  KF_DEBUG(fn, "Start");

  /**
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_ * x_;
  KF_DEBUG(fn, "End 1");
  
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  KF_DEBUG(fn, "End 2");
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  
  KF_DEBUG(fn, "End 3");
  
  x_ = x_ + K * y;
  P_ = (I_ - K * H_) * P_;
  
  KF_DEBUG(fn, "End 4");
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

  fn = "UpdateEFK";
  KF_DEBUG(fn, "Start");
  KF_DEBUG(fn, x_);

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
  KF_DEBUG(fn, "rho");  
  KF_DEBUG(fn, rho);
  
  VectorXd z_pred(3);
  z_pred << rho, atan2( py, px ), ( px*vx + py*vy )/rho;

  KF_DEBUG(fn, "z_pred");  
  KF_DEBUG(fn, z_pred);
  
  // Update the state using Extended Kalman Filter equations
  VectorXd y = z - z_pred;
  if( y[1] > PI )
    y[1] -= 2.f*PI;
  if( y[1] < -PI )
    y[1] += 2.f*PI;

  KF_DEBUG(fn, "y");  
  KF_DEBUG(fn, y);
  
  MatrixXd Hj_ = tools.CalculateJacobian( x_ );
  KF_DEBUG(fn, "Hj_");  
  KF_DEBUG(fn, Hj_);
  
  KF_DEBUG(fn, "P_");  
  KF_DEBUG(fn, P_);
  KF_DEBUG(fn, "R_");  
  KF_DEBUG(fn, R_);
  
  MatrixXd S = Hj_*P_* Hj_.transpose() + R_;
  KF_DEBUG(fn, "S");  
  KF_DEBUG(fn, S);
  
  MatrixXd K =  P_*Hj_.transpose()*S.inverse();
  KF_DEBUG(fn, "K");  
  KF_DEBUG(fn, K);
  
  // Compute new state
  x_ = x_ + ( K * y );
  P_ = ( I_ - K * Hj_ ) * P_;
}
