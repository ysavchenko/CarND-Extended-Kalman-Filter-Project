#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()
  		|| estimations.size() == 0){
  	cout << "Invalid estimation or ground_truth data" << endl;
  	return rmse;
  }

  //accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){

  	VectorXd residual = estimations[i] - ground_truth[i];

  	//coefficient-wise multiplication
  	residual = residual.array()*residual.array();
  	rmse += residual;
  }

  //calculate the mean
  rmse = rmse/estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	float sum_squares = px * px + py * py;
	//check division by zero
	if (sum_squares == 0) {
	    cout << "Division by zero" << endl;
	    return Hj;
	}
	float sum_squares_sqrt = sqrt(sum_squares);
	float sum_squares_3_2 = sum_squares * sum_squares_sqrt;
	float vxpy_vypx = vx * py - vy * px;
	float vypx_vxpy = vy * px - vx * py;
	float px_squares_sqrt = px / sum_squares_sqrt;
	float py_squares_sqrt = py / sum_squares_sqrt;
	
	//compute the Jacobian matrix
	Hj << px_squares_sqrt, py_squares_sqrt, 0, 0,
	    -py / sum_squares, px / sum_squares, 0, 0,
	    py * vxpy_vypx / sum_squares_3_2, px * vypx_vxpy / sum_squares_3_2,
	    px_squares_sqrt, py_squares_sqrt;

	return Hj;
}
