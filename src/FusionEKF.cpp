#include <iostream>
#include <math.h>
#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"

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
  Hj_ = MatrixXd(3, 4);

  // the Kalman Filter instance
  ekf_ = KalmanFilter();

  // measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // measurement function matrix - laser
  H_laser_ << 1.0, 0, 0, 0,
              0, 1.0, 0, 0;

  // initialization of kalman filter instance
  VectorXd x = VectorXd::Zero(4);  // the mix of position and velocity
  MatrixXd P = MatrixXd::Identity(4, 4);  // the state covariance
  MatrixXd F = MatrixXd::Identity(4, 4);  // the state transition function
  // we set delta_t to 1 initially
  F(0, 2) = 1.0;
  F(1, 3) = 1.0;
  MatrixXd Q = MatrixXd::Zero(4, 4); // TODO: I think this step should go in the Tracking constructor

  // then pass it all to the kalman filter
  ekf_.Init(x, P, F, H_laser_, R_laser_, Q);

  // measurement function matrix (Jacobian) - radar
  //Hj_ = tools.CalculateJacobian(ekf_.x_);

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  double dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000.;
  previous_timestamp_ = measurement_pack.timestamp_;

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double ro = measurement_pack.raw_measurements_[0];
      double theta = measurement_pack.raw_measurements_[1];
      double ro_dot = measurement_pack.raw_measurements_[2];
      double theta_cos = cos(theta);
      double theta_sin = sin(theta);
      //ekf_.x_ << ro * theta_cos, ro * theta_sin, ro_dot * theta_cos, ro_dot * theta_sin;
      ekf_.x_ << ro * theta_cos, ro * theta_sin, 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0],
                 measurement_pack.raw_measurements_[1],
                 0,
                 0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  ekf_.F_(0, 2) = ekf_.F_(1, 3) = dt;

  const float noise_ax = 9;
  const float noise_ay = 9;

  double dt_4_by_4 = pow(dt, 4.0)/4.0;
  double dt_3_by_2 = pow(dt, 3.0)/2.0;
  double dt_2 = pow(dt, 2);
  ekf_.Q_ << dt_4_by_4*noise_ax, 0, dt_3_by_2*noise_ax, 0,
            0, dt_4_by_4*noise_ay, 0, dt_3_by_2*noise_ay,
            dt_3_by_2*noise_ax, 0, dt_2*noise_ax, 0,
            0, dt_3_by_2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
