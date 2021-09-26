#include "FusionUKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionUKF::FusionUKF() 
{
  is_initialized_ = false;

  previous_timestamp_ = 0;
  count = 0;

  // initializing matrices
  ukf_.x_ = VectorXd(5);
  ukf_.P_ = MatrixXd(5, 5);
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 5);

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  //Initialize the state x_
  ukf_.x_ << 0,0,0,0,0;
  
  //Initialize the Covariance matrix P_
  ukf_.P_ << 1, 0, 0, 0, 0,
             0, 1, 0, 0, 0,
             0, 0, 1, 0, 0,
             0, 0, 0, 1, 0,
			 0, 0, 0, 0, 1;
  
  //Initialize the H Matrix for LiDAR:
  H_laser_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0;
  
  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  
  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

}

/**
 * Destructor.
 */
FusionUKF::~FusionUKF() {}

void FusionUKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
  /**
   * Initialization
   */
  if (!is_initialized_) 
  {
    /**
     * TODO: Initialize the state ukf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "UKF: Initialization " << endl;
    previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
    {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      cout << "RADAR Initialization " << endl;
      ukf_.x_ << measurement_pack.raw_measurements_[0]*cos(measurement_pack.raw_measurements_[1]),
                 measurement_pack.raw_measurements_[0]*sin(measurement_pack.raw_measurements_[1]),
                 0,
                 0,
				 0;
      
      ukf_.R_ = R_radar_;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) 
    {
      // TODO: Initialize state.
      cout << "Laser Initialization " << endl;
      ukf_.x_ << measurement_pack.raw_measurements_[0],
                 measurement_pack.raw_measurements_[1],
                 0,
                 0,
				 0;
      
      ukf_.H_ = H_laser_;
      ukf_.R_ = R_laser_;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    cout << "UKF: Initialization compleated" << endl;
    //return;
  }
  else
  {

    /**
     * Prediction
     */

    /**
     * TODO: Update the state transition matrix F according to the new elapsed time.
     * Time is measured in seconds.
     * TODO: Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */
    // compute the time elapsed between the current and previous measurements
    // dt - expressed in seconds
    ukf_.dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    /**
     * Predict Function
     */
    cout << "Prediction step: " << count << endl;

    ukf_.AugmentedSigmaPoints();
	ukf_.SigmaPointPrediction();
	ukf_.PredictMeanAndCovariance();

    /**
     * Update Functions
     */

    /**
     * TODO:
     * - Use the sensor type to perform the update step.
     * - Update the state and covariance matrices.
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
    {
      // TODO: Radar updates
      ukf_.R_ = R_radar_;
      cout << "RADAR Measurement update step: " << count << endl;
	  ukf_.PredictRadarMeasurement();
      ukf_.UpdateStateRadar(measurement_pack.raw_measurements_);
    }
    else 
    {
      // TODO: Laser updates
      ukf_.H_ = H_laser_;
      ukf_.R_ = R_laser_;
      cout << "LASER Measurement update step: " << count << endl;
      ukf_.UpdateStateLaser(measurement_pack.raw_measurements_);
    }
  }
  count++;
  // print the output
  cout << "x_ = " << ukf_.x_ << endl;
  cout << "P_ = " << ukf_.P_ << endl;
}