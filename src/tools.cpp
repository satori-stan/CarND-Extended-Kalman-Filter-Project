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
  rmse << 0, 0, 0, 0;

  if (estimations.size() == 0) {
    cout << "ERROR: The estimations vector is empty" << endl;
  } else {
    if (estimations.size() != ground_truth.size()) {
      cout << "ERROR: The number of estimations and ground truths is different"
          << endl;
    } else {
      uint32_t size = estimations.size();
      for (uint32_t i = 0; i < size; ++i) {
        rmse += static_cast<VectorXd>(
            (estimations[i] - ground_truth[i]).array().pow(2));
      }
      rmse /= size;
      rmse = rmse.array().sqrt();
    }
  }
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3, 4);
  //recover state parameters
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  double common_divisor = px*px + py*py;
  // check division by zero
  if (common_divisor != 0) {
    // compute the Jacobian matrix
    Hj(0, 0) = Hj(2, 2) = px / sqrt(common_divisor);
    Hj(0, 1) = Hj(2, 3) = py / sqrt(common_divisor);
    Hj(1, 0) = -(py/common_divisor);
    Hj(1, 1) = px/common_divisor;
    Hj(2, 0) = py*((vx*py)-(vy*px))/pow(common_divisor, 3/2.F);
    Hj(2, 1) = px*((vy*px)-(vx*py))/pow(common_divisor, 3/2.F);
  } else {
    cout << "Error with division by zero when calculating the Jacobian." << endl;
  }
  return Hj;
}
