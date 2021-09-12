#include "kalman_filter.h"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() 
{
  z_pred_radar_ = VectorXd(3);
  
  //Initialize the vector:
  z_pred_radar_ << 0,0,0;
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) 
{
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() 
{
  /**
   * Update the state variables..
   */
  x_ = F_ * x_;
  MatrixXd Ft_ = F_.transpose();
  P_ = (F_ * P_ * Ft_) + Q_;
}

void KalmanFilter::Update(const VectorXd &z) 
{
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  
  //calculate the matrices..
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht_ = H_.transpose();
  MatrixXd S_ = (H_ * P_ * Ht_) + R_;
  MatrixXd SI_ = S_.inverse();
  MatrixXd PHt_ = P_ * Ht_;
  MatrixXd K_ = PHt_ * SI_;
  
  //new measurement Update..
  x_ = x_ + (K_ * y);
  long x_size = x_.size();
  MatrixXd I_ = MatrixXd::Identity(x_size, x_size);
  P_ = (I_ - (K_ * H_)) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) 
{
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  
  //transform from Cartesian to Polar Coords..
  z_pred_radar_[0] = sqrt((x_[0]*x_[0]) + (x_[1]*x_[1]));
  z_pred_radar_[1] = atan2(x_[1],x_[0]);
  z_pred_radar_[2] = ((x_[0]*x_[2])+(x_[1]*x_[3]))/(sqrt((x_[0]*x_[0]) + (x_[1]*x_[1])));
  
  //calculate the matrices..
  VectorXd y = z - z_pred_radar_;
  while (y(1) > M_PI)
  {
    y(1) -= 2.*M_PI;
  }
  while (y(1) < -M_PI)
  {
    y(1) += 2.*M_PI;
  }
  MatrixXd Ht_ = H_.transpose();
  MatrixXd S_ = (H_ * P_ * Ht_) + R_;
  MatrixXd SI_ = S_.inverse();
  MatrixXd PHt_ = P_ * Ht_;
  MatrixXd K_ = PHt_ * SI_;
  
  //new measurement Update..
  x_ = x_ + (K_ * y);
  long x_size = x_.size();
  MatrixXd I_ = MatrixXd::Identity(x_size, x_size);
  P_ = (I_ - (K_ * H_)) * P_;
}
