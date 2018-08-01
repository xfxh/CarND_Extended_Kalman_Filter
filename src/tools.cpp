#include <iostream>
#include "tools.h"
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  if (estimations.size()==0)
    {
        cout<<"the estimation vector size should not be zero"<<endl;
        return rmse;
    }
  if (estimations.size() != ground_truth.size())
    {
        cout<<"the estimation vector size should equal ground truth vector size"<<endl;
        return rmse;
    }

  //accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){
    //coefficient-wise multiplication
     VectorXd residual = estimations[i] - ground_truth[i];
              residual = residual.array()*residual.array();
		 rmse += residual;
	}

  //calculate the mean
  rmse = rmse/estimations.size();
  //calculate the squared root
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //check division by zero
  if ((px*px+py*py) == 0)
  { 
     cout<<"division by zero";
  }
  //compute the Jacobian matrix
  else
  {
     Hj<<px/sqrt(px*px+py*py), py/sqrt(px*px+py*py), 0, 0,
	         -py/(px*px+py*py), px/(px*px+py*py), 0, 0,
	          py*(vx*py-vy*px)/pow(px*px+py*py,1.5), px*(vy*px-vx*py)/pow(px*px+py*py,1.5),px/sqrt(px*px+py*py),py/sqrt(px*px+py*py);
  } 
  
  return Hj;
}
