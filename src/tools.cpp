#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd>& estimations,
                              const vector<VectorXd>& ground_truth) {

  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
    return rmse;
  }

  for (unsigned int i = 0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  * Calculate a Jacobian here.
  */
  MatrixXd Hj(3, 4);

  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float h1 = px * px + py * py;
  float h2 = sqrt(h1);
  float h3 = vx * py - vy * vx;

  if (fabs(h1) < 0.0001) {
    Hj << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;
    return Hj;
  }

  Hj << (px / h2), (py / h2), 0, 0,
      -(py / h1), (px / h1), 0, 0,
      py * h3 / (h1 * h2), -(px * h3 / (h1 * h2)), px / h2, py / h2;

  return Hj;

}

VectorXd Tools::CalculateRMSEContinuous(const Eigen::VectorXd& estimations,
                                        const Eigen::VectorXd& ground_truth,
                                        const Eigen::VectorXd& rmse,
                                        const int message_count) {
  /**
  * Update the RMSE with new measurement and ground truth.
  */
  VectorXd residual = estimations - ground_truth;
  residual = residual.array() * residual.array();
  VectorXd sum = rmse.array() * rmse.array() * message_count + residual.array();

  return (sum.array() / (message_count + 1)).sqrt();
}

Struct Tools::PreprocessPackages(std::string data) {
  Struct packages;

  std::string a(data);
  std::istringstream iss(a);

  std::string sensor_type;
  iss >> sensor_type;
  long timestamp;

  if (sensor_type.compare("L") == 0) {
    // LASER MEASUREMENT

    packages.meas.sensor_type_ = MeasurementPackage::LASER;
    packages.meas.raw_measurements_ = VectorXd(2);
    float x;
    float y;
    iss >> x;
    iss >> y;
    packages.meas.raw_measurements_ << x, y;
    iss >> timestamp;
    packages.meas.timestamp_ = timestamp;

  } else if (sensor_type.compare("R") == 0) {
    // RADAR MEASUREMENT

    packages.meas.sensor_type_ = MeasurementPackage::RADAR;
    packages.meas.raw_measurements_ = VectorXd(3);
    float ro;
    float phi;
    float ro_dot;
    iss >> ro;
    iss >> phi;
    iss >> ro_dot;
    packages.meas.raw_measurements_ << ro, phi, ro_dot;
    iss >> timestamp;
    packages.meas.timestamp_ = timestamp;
  }
  // Read ground truth data.
  float x_gt;
  float y_gt;
  float vx_gt;
  float vy_gt;
  iss >> x_gt;
  iss >> y_gt;
  iss >> vx_gt;
  iss >> vy_gt;
  packages.gt.gt_values_ = VectorXd(4);
  packages.gt.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;

  return packages;
}