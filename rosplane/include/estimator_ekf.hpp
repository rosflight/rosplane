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
                                                                  Eigen::VectorXf u,
                                                                  Eigen::VectorXf h,
                                                                  Eigen::VectorXf y,
                                                                  Eigen::MatrixXf R,
                                                                  Eigen::MatrixXf C,
                                                                  Eigen::MatrixXf P,
                                                                  float Ts);        
  std::tuple<Eigen::MatrixXf, Eigen::VectorXf> propagate_model(Eigen::VectorXf x,
                                                               Eigen::VectorXf f,
                                                               Eigen::MatrixXf A,
                                                               Eigen::MatrixXf P,
                                                               Eigen::MatrixXf G,
                                                               Eigen::MatrixXf Q_g,
                                                               Eigen::MatrixXf Q,
                                                               float Ts);          

private:
  virtual void estimate(const input_s & input, output_s & output) override = 0;
  void check_measurment_update_input(Eigen::VectorXf x,
                                     Eigen::VectorXf u,
                                     Eigen::VectorXf h,
                                     Eigen::VectorXf y,
                                     Eigen::MatrixXf R,
                                     Eigen::MatrixXf C,
                                     Eigen::MatrixXf P,
                                     float Ts);        
  void check_propagate_model_input(Eigen::VectorXf x,
                                     Eigen::VectorXf f,
                                     Eigen::MatrixXf A,
                                     Eigen::MatrixXf P,
                                     Eigen::MatrixXf G,
                                     Eigen::MatrixXf Q_g,
                                     Eigen::MatrixXf Q,
                                     float Ts);          
}; 

} // namespace rosplane

#endif // ESTIMATOR_EKF_H
