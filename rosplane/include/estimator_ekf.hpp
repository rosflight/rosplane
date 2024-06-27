#ifndef ESTIMATOR_EKF_H
#define ESTIMATOR_EKF_H

#include "estimator_ros.hpp"
#include <Eigen/Geometry>
#include <math.h>
#include <tuple>
#include <cassert>
#include <yaml-cpp/yaml.h>

namespace rosplane
{

class estimator_ekf : public estimator_ros
{
public:
  estimator_ekf();

protected:
  std::tuple<Eigen::MatrixXf, Eigen::VectorXf> measurement_update(Eigen::VectorXf x,
                                                                  std::function<Eigen::VectorXf(Eigen::VectorXf)> measurement_model,
                                                                  Eigen::VectorXf y,
                                                                  std::function<Eigen::MatrixXf(Eigen::VectorXf)> measurement_jacobian,
                                                                  Eigen::MatrixXf R,
                                                                  Eigen::MatrixXf P);
  std::tuple<Eigen::MatrixXf, Eigen::VectorXf> propagate_model(Eigen::VectorXf x,
                                                               std::function<Eigen::VectorXf(Eigen::VectorXf)> dynamic_model,
                                                               std::function<Eigen::MatrixXf(Eigen::VectorXf)> jacobian,
                                                               Eigen::MatrixXf Q_g,
                                                               std::function<Eigen::MatrixXf(Eigen::VectorXf)> input_jacobian,
                                                               Eigen::MatrixXf P,
                                                               Eigen::MatrixXf Q,
                                                               float Ts);          

private:
  virtual void estimate(const input_s & input, output_s & output) override = 0;
}; 

} // namespace rosplane

#endif // ESTIMATOR_EKF_H
