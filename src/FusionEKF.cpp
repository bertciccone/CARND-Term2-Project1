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
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
  0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
  0, 0.0009, 0,
  0, 0, 0.09;

  H_laser_ << // From lesson 5, section 10
    1, 0, 0, 0,
    0, 1, 0, 0;

  float dt = 0; // initialize to 0 until first measurement is available
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << // 4x4 matrix (state transition), lesson 5, section 12
    1, 0, dt, 0,
    0, 1, 0,  dt,
    0, 0, 1,  0,
    0, 0, 0,  1;

  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << // TODO: 4x4 matrix (covariance); not covered in walk through, assume 0's for now
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0;

  // Acceleration noise components (section 13)
  noise_ax_ = 3 * 3; // Provided in lesson 5, section 13; 3 * 3 = 9
  noise_ay_ = noise_ax_; // Assume noise_ay == noise_ax

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
    ekf_.x_ <<
      1, 1, // these values will be overwritten below
      4, 0; // TODO: these values need to be tweaked for correct RMSE

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
       Convert radar from polar to cartesian coordinates and initialize state.
       */
      // TODO: ro is raw_measurements[0], theta is raw_measurements[1]
      // TODO: set ekf_.x_[0] to ro*cos(theta)
      ekf_.x_[0] =
        measurement_pack.raw_measurements_[0] *
        cos(measurement_pack.raw_measurements_[1]);
      // TODO: set ekf_.x_[1] to ro*sin(theta)
      ekf_.x_[1] =
        measurement_pack.raw_measurements_[0] *
        sin(measurement_pack.raw_measurements_[1]);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
       Initialize state.
       */
      // TODO: set ekf_.x_[0] to x or raw_measurements[0]
      ekf_.x_[0] = measurement_pack.raw_measurements_[0];
      // TODO: set ekf_.x_[1] to y or raw_measurements[1]
      ekf_.x_[1] = measurement_pack.raw_measurements_[1];
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    ekf_.F_ << // TODO: set to 1 diagonal matrix
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
   * Update the state transition matrix F according to the new elapsed time.
   - Time is measured in seconds.
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // Update time in the F matrix, from lesson 5, section 8
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // Updated the process covariance matrix Q, from quiz in lesson 5, section 9

  float dt2 = dt*dt;
  float dt3 = dt2*dt;
  float dt4 = dt3*dt;

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<
  dt4/4*noise_ax_,  0,                dt3/2*noise_ax_,  0,
  0,                dt4/4*noise_ay_,  0,                dt3/2*noise_ay_,
  dt3/2*noise_ax_,  0,                dt2*noise_ax_,    0,
  0,                dt3/2*noise_ay_,  0,                dt2*noise_ay_;







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

    // set ekf_.H by setting to Hj which is the calculated Jacobian
    Tools tools;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    // set ekf_.R by using R_radar_
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates

    // set ekf_.H by just using H_laser_
    ekf_.H_ = H_laser_;
    // set ekf_.R by just using R_laser_
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // ekf_.Update(measurement_pack.raw_measurements_);

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
