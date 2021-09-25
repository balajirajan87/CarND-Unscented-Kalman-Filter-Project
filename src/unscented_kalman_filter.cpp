#include "unscented_kalman_filter.h"
#include <cmath>
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

UKalmanFilter::UKalmanFilter() 
{
  z_pred_radar_ = VectorXd(3);
  
  //Initialize the vector:
  z_pred_radar_ << 0,0,0;
}

UKalmanFilter::~UKalmanFilter() {}

void UKalmanFilter::AugmentedSigmaPoints()
{
	n_x = 5;
	n_aug = 7;
	std_a = 3;
	std_yawdd = 0.2;
	lambda = 3 - n_aug;

	// create augmented state covariance
	MatrixXd P_aug = MatrixXd(n_aug, n_aug);

	// create sigma point matrix
	Xsig_aug_ = MatrixXd(n_aug, 2 * n_aug + 1);

	// create augmented mean state
	VectorXd x_mean = VectorXd(n_aug);
	x_mean << 0,0,0,0,0,0,0;
	x_mean.head(5) = x_;

	// create augmented covariance matrix
	Q_ = MatrixXd(2, 2);
	Q_ <<      std_a*std_a, 0,
              0, std_yawdd*std_yawdd;

	P_aug <<  0,0,0,0,0,0,0,
			  0,0,0,0,0,0,0,
              0,0,0,0,0,0,0,
              0,0,0,0,0,0,0,
              0,0,0,0,0,0,0,
              0,0,0,0,0,0,0,
              0,0,0,0,0,0,0;

	P_aug.topLeftCorner(5, 5) = P_;
    P_aug.bottomRightCorner(2, 2) = Q_;

    // create square root matrix
	MatrixXd A = P_aug.llt().matrixL();

	// create augmented sigma points
	Xsig_aug_.col(0) = x_mean;

	// set remaining sigma points
	for (int i = 0; i < n_aug; ++i)
	{
		Xsig_aug_.col(i+1)     = x_mean + sqrt(lambda+n_aug) * A.col(i);
		Xsig_aug_.col(i+1+n_aug) = x_mean - sqrt(lambda+n_aug) * A.col(i);
	}
}

void UKalmanFilter::SigmaPointPrediction()
{
    // create matrix with predicted sigma points as columns
	Xsig_pred_ = MatrixXd(n_x, 2 * n_aug + 1);

	VectorXd x_point = VectorXd(7);
	VectorXd vec1 = VectorXd(5);
	VectorXd vec2 = VectorXd(5);

	float delta_t = dt;
  
	// predict sigma points
	for (int i=0; i<(2 * n_aug + 1); i++)
	{
		x_point = Xsig_aug_.col(i);
		vec2 <<    (0.5*delta_t*delta_t*cos(x_point(3))*x_point(5)),
				   (0.5*delta_t*delta_t*sin(x_point(3))*x_point(5)),
				   (delta_t*x_point(5)),
				   (0.5*delta_t*delta_t*x_point(6)),
				   (delta_t*x_point(6));
                   
		if (x_point(4) != 0)
		{
			vec1 <<  (x_point(2)/x_point(4))*(sin(x_point(3)+x_point(4)*delta_t)-sin(x_point(3))),
					 (x_point(2)/x_point(4))*(-cos(x_point(3)+x_point(4)*delta_t)+cos(x_point(3))),
					 0,
				     x_point(4)*delta_t,
				     0;

			Xsig_pred_.col(i) = x_point.head(5) + vec1 + vec2;
		}
		else
		{
			vec1 <<  x_point(2)*cos(x_point(3))*delta_t,
				     x_point(2)*sin(x_point(3))*delta_t,
				     0,
				     x_point(4)*delta_t,
				     0;
                   
			Xsig_pred_.col(i) = x_point.head(5) + vec1 + vec2;
		}
	}
}

void UKalmanFilter::PredictMeanAndCovariance() 
{

  // create vector for weights
  weights_ = VectorXd(2*n_aug+1);
  weights_ <<  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
  
  VectorXd x_diff = VectorXd(n_x);

  /**
   * Student part begin
   */

  // set weights
  for (int i=0; i<(2*n_aug+1); i++)
  {
      if(i==0)
      {
          weights_(i) = lambda / (lambda + n_aug);
      }
      else
      {
          weights_(i) = 1 / (2*(lambda + n_aug)); 
      }
  }
  
  // predict state mean
  for (int i=0; i<(2*n_aug+1); i++)
  {
      x_ = x_ + (weights_(i)*Xsig_pred_.col(i));
  }
  
  // predict state covariance matrix
  for (int i=0; i<(2*n_aug+1); i++)
  {
      x_diff = Xsig_pred_.col(i)-x_;
      while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
      while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
      P_ = P_ + (weights_(i)*(x_diff*x_diff.transpose()));
  }

  /**
   * Student part end
   */

  // print result
  std::cout << "Predicted state" << std::endl;
  std::cout << x_ << std::endl;
  std::cout << "Predicted covariance matrix" << std::endl;
  std::cout << P_ << std::endl;
}

void UKalmanFilter::PredictRadarMeasurement() 
{

  // set measurement dimension, radar can measure r, phi, and r_dot
  n_z = 3;

  // create matrix for sigma points in measurement space
  Zsig_ = MatrixXd(n_z, 2 * n_aug + 1);

  // mean predicted measurement
  VectorXd z_pred_radar_ = VectorXd(n_z);
  
  // measurement covariance matrix S
  MatrixXd S_ = MatrixXd(n_z,n_z);

  VectorXd z_diff = VectorXd(n_z);

  /**
   * Student part begin
   */
  // transform sigma points into measurement space
  VectorXd x_vec = VectorXd(n_x);
  VectorXd z_vec = VectorXd(n_z);
  for (int i=0; i<(2 * n_aug + 1); i++)
  {
      x_vec = Xsig_pred_.col(i);
      z_vec << sqrt(x_vec(0)*x_vec(0) + x_vec(1)*x_vec(1)),
               atan2(x_vec(1), x_vec(0)),
               (x_vec(0)*x_vec(2)*cos(x_vec(3)) + x_vec(1)*x_vec(2)*sin(x_vec(3)))/(sqrt(x_vec(0)*x_vec(0) + x_vec(1)*x_vec(1)));

      Zsig_.col(i) = z_vec;
  }
  // calculate mean predicted measurement
  for (int i=0; i<(2*n_aug+1); i++)
  {
      z_pred_radar_ = z_pred_radar_ + (weights_(i)*Zsig_.col(i));
  }
  // calculate innovation covariance matrix S
  for (int i=0; i<(2*n_aug+1); i++)
  {
      z_diff = Zsig_.col(i)-z_pred_radar_;
      while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
      while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
      S_ = S_ + (weights_(i)*(z_diff*z_diff.transpose()));
  }

  S_ = S_ + R_;
  /**
   * Student part end
   */

  // print result
  std::cout << "z_pred: " << std::endl << z_pred_radar_ << std::endl;
  std::cout << "S: " << std::endl << S_ << std::endl;
}

void UKalmanFilter::UpdateStateRadar(const VectorXd &z) 
{

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x, n_z);

  /**
   * Student part begin
   */

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig_.col(i) - z_pred_radar_;
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S_.inverse();

  // residual
  VectorXd z_diff = z - z_pred_radar_;

  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S_*K.transpose();

  /**
   * Student part end
   */
}


void UKalmanFilter::UpdateStateLaser(const VectorXd &z) 
{

  
  //calculate the matrices..
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht_ = H_.transpose();
  MatrixXd Slaser_ = (H_ * P_ * Ht_) + R_;
  MatrixXd SI_ = Slaser_.inverse();
  MatrixXd PHt_ = P_ * Ht_;
  MatrixXd K_ = PHt_ * SI_;
  
  //new measurement Update..
  x_ = x_ + (K_ * y);
  long x_size = x_.size();
  MatrixXd I_ = MatrixXd::Identity(x_size, x_size);
  P_ = (I_ - (K_ * H_)) * P_;
}
