#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) 
{
  /**
   * TODO: Calculate the RMSE here.
   */
  //initialize the variables..
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  VectorXd error(4);
  error << 0,0,0,0;
  VectorXd eSqr(4);
  eSqr << 0,0,0,0;
  VectorXd eSqrMean(4);
  eSqrMean << 0,0,0,0;
  
  // TODO: YOUR CODE HERE
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != 0 && estimations.size() == ground_truth.size())
  {
      // TODO: accumulate squared residuals
      for (int i=0; i < estimations.size(); ++i) 
      {
        // ... your code here
        error = estimations[i] - ground_truth[i];
        eSqr = error.array()*error.array();
        eSqrMean = eSqrMean + eSqr;
      }
    
      // TODO: calculate the mean
      eSqrMean = eSqrMean.array()/estimations.size();
      // TODO: calculate the squared root
      rmse = eSqrMean.array().sqrt();
      // return the result
  }
  else
  {
      cout << "Invalid estimation or ground_truth data" << endl;
      rmse << 0,0,0,0;
  }
  return rmse;
}
/*
double Tools::CalculateNISScore(const VectorXd &z_state, const VectorXd &z_meas, const MatrixXd &S) 
{
  VectorXd z_diff(3);
  z_diff = z_meas - z_state;
  VectorXd z_diff_T = z_diff.transpose();
  double nisscore = z_diff_T * S.inverse() * z_diff;
}
*/