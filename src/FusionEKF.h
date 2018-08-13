#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "kalman_filter.h"
#include "Eigen/Dense"
#include <fstream>
#include <vector>
#include <string>

#ifdef DEBUG
#define FUSION_DEBUG(fn, log) std::cout << "FusionEKF " << fn << " : " << log << "\n";
#else
#define FUSION_DEBUG(fn, log)
#endif

class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter ekf_;

private:
  string fn;

  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;

  float noise_ax;
  float noise_ay;
};

#endif /* FusionEKF_H_ */
