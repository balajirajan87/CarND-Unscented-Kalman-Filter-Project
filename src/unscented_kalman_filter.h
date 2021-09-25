#ifndef UNSCENTED_KALMAN_FILTER_H_
#define UNSCENTED_KALMAN_FILTER_H_

#include "Eigen/Dense"

class UKalmanFilter {
 public:
  /**
   * Constructor
   */
  UKalmanFilter();

  /**
   * Destructor
   */
  virtual ~UKalmanFilter();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateStateRadar(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateStateLaser(const Eigen::VectorXd &z);

  /**
   * Generates the Augmented sigma points based on the State and the covariance matrix
   */
  void AugmentedSigmaPoints();

  /**
   * Predicts the next state for each and every sigma points generated..
   */
  void SigmaPointPrediction();

  /**
   * Predicts the mean and covariance of the next predicted state k+1.
   */
  void PredictMeanAndCovariance();

  /**
   * used to transform the sigma points to the measurements space for RADAR.
   */
  void PredictRadarMeasurement();

  // state vector
  Eigen::VectorXd x_;

  // create vector for weights
  Eigen::VectorXd weights_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix only for Lidar ?
  Eigen::MatrixXd H_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;
  
  //Radar Prediction Vector from Cartesian Coords..
  Eigen::VectorXd z_pred_radar_;

  // mean predicted measurement (Laser ?)
  Eigen::VectorXd z_pred_;

  //Define the Augmented Sigma points
  Eigen::MatrixXd Xsig_aug_;

  // create matrix with predicted sigma points as columns
  Eigen::MatrixXd Xsig_pred_;

  // measurement covariance matrix S
  Eigen::MatrixXd S_;

  // create matrix for sigma points in measurement space
  Eigen::MatrixXd Zsig_;

  // set state dimension
  int n_x;

  // set measurement dimension
  int n_z;

  // set augmented dimension
  int n_aug;

  //Process noise Std Deviation Long Accel m/s^2.
  double std_a;

  //Process noise std Dev yaw accel: rad/s^2:
  double std_yawdd;

  //define spreading parameter
  double lambda;

  //define the delta time
  float dt;

  
};

#endif // UNSCENTED_KALMAN_FILTER_H_
