#include "FusionEKF.h"
#include "Eigen/Dense"
#include <iostream>
#include <string>

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
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  //measurement matrix - laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
  /**
    * Finish initializing the FusionEKF.
    * VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in
    */
  VectorXd v = VectorXd(4);
  MatrixXd m = MatrixXd(4, 4);

  v << 0, 0, 0, 0;
  m << 0, 0, 0, 0,
       0, 0, 0, 0,
       0, 0, 0, 0, 
       0, 0, 0, 0;

  ekf_.Init( v, m, m, m, m , m);
  /**
    * Set the process and measurement noises
    **/
  noise_ax = 9;
  noise_ay = 9;

  /*
   * private variable for debug
   */
  fn = "Constructor";
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  fn = "ProcessMeasurement";
 
  FUSION_DEBUG(fn, "Start");
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    FUSION_DEBUG(fn, "initializ");
    /**
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    //ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      FUSION_DEBUG(fn, "RADAR");
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float ro;    // radical distance from origin 
      float theta;  // angle between ro and x axis
      float ro_dot; // radical volocity 

      ro = measurement_pack.raw_measurements_[0];  
      theta = measurement_pack.raw_measurements_[1];  
      ro_dot = measurement_pack.raw_measurements_[2];  

      ekf_.x_ << ro * cos(theta), ro * sin(theta), 0, 0;  // x, y, vx, vy
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      FUSION_DEBUG(fn, "LASER");
      /**
      Initialize state.
      */
      float px; 
      float py;

      px = measurement_pack.raw_measurements_[0];  
      py = measurement_pack.raw_measurements_[1];  

      ekf_.x_ << px, py, 0, 0;
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  FUSION_DEBUG(fn, "Prediction 1");

  // Compute the time from the previous measurement in seconds.
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
	previous_timestamp_ = measurement_pack.timestamp_;
  
  FUSION_DEBUG(fn, "Prediction 2");
  FUSION_DEBUG(fn, previous_timestamp_);
  FUSION_DEBUG(fn, ekf_.F_);

  // Update the motion model matrix for a timestep dt.
  // We use a motion model with a constant velocity.
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;
  
  FUSION_DEBUG(fn, "Prediction 3");
  FUSION_DEBUG(fn, dt);
  // Update the process noise covariance matrix for a timestep dt.
  // Our motion model uses Gaussian random accelerations in the x and y directions.
  float dt2 = dt*dt;
  float dt3 = dt2*dt;
  float dt4 = dt3*dt;
  float dt4over4 = dt4/4.;
  float dt3over2 = dt3/2.;

  FUSION_DEBUG(fn, "Prediction 4");

  ekf_.Q_ << dt4over4*noise_ax,                 0, dt3over2*noise_ax,                0,
			     0, dt4over4*noise_ay,                 0, dt3over2*noise_ay,
	     dt3over2*noise_ax,                 0,      dt2*noise_ax,                 0,
			     0, dt3over2*noise_ay,                 0,      dt2*noise_ay;

  FUSION_DEBUG(fn, "Prediction 5 x ");
  FUSION_DEBUG(fn, ekf_.x_);
  FUSION_DEBUG(fn, "Prediction 5 F ");
  FUSION_DEBUG(fn, ekf_.F_);
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/
 FUSION_DEBUG(fn, "Update");
 FUSION_DEBUG(fn, measurement_pack.sensor_type_);
  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    FUSION_DEBUG(fn, "Update Radar");
    // Radar updates
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF( measurement_pack.raw_measurements_ );
  } else {
    FUSION_DEBUG(fn, "Update Ladar");
    // Laser updates    
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update( measurement_pack.raw_measurements_ );
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
