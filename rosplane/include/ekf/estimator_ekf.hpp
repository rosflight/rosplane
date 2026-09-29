#ifndef ESTIMATOR_EKF_H
#define ESTIMATOR_EKF_H

#include <tuple>

#include <Eigen/Geometry>

#include "estimator_ros.hpp"

// These declutter code, and incur a minimum time cost when passing these types to functions.
using DynamicModelFuncRef =
  std::function<Eigen::VectorXd(const Eigen::VectorXd, const Eigen::VectorXd)>;
using MeasurementModelFuncRef =
  std::function<Eigen::VectorXd(const Eigen::VectorXd, const Eigen::VectorXd)>;
using JacobianFuncRef =
  std::function<Eigen::MatrixXd(const Eigen::VectorXd, const Eigen::VectorXd)>;
using SensorNoiseFuncRef =
  std::function<Eigen::MatrixXd(const Eigen::VectorXd, const Eigen::VectorXd)>;

namespace rosplane
{

class EstimatorEKF : public EstimatorROS
{
public:
  EstimatorEKF();

protected:
  std::tuple<Eigen::MatrixXd, Eigen::VectorXd> kalman_update(Eigen::VectorXd x, Eigen::VectorXd h,
                                                             Eigen::VectorXd y, Eigen::MatrixXd C,
                                                             Eigen::MatrixXd R, Eigen::MatrixXd S,
                                                             Eigen::MatrixXd P);

  std::tuple<Eigen::MatrixXd, Eigen::VectorXd>
  measurement_update(Eigen::VectorXd x, Eigen::VectorXd inputs,
                     MeasurementModelFuncRef measurement_model, Eigen::VectorXd y,
                     JacobianFuncRef measurement_jacobian, SensorNoiseFuncRef sensor_noise_model,
                     Eigen::MatrixXd P);

  std::tuple<Eigen::MatrixXd, Eigen::VectorXd>
  calculate_measurement_update(Eigen::VectorXd x, Eigen::VectorXd inputs, Eigen::MatrixXd h,
                               Eigen::VectorXd y, Eigen::MatrixXd C, Eigen::MatrixXd R,
                               Eigen::MatrixXd P);

  std::tuple<Eigen::MatrixXd, Eigen::VectorXd>
  propagate_model(Eigen::VectorXd x, DynamicModelFuncRef dynamic_model, JacobianFuncRef jacobian,
                  Eigen::VectorXd inputs, JacobianFuncRef input_jacobian, Eigen::MatrixXd P,
                  Eigen::MatrixXd Q, Eigen::MatrixXd Q_g, double Ts);

  std::tuple<Eigen::MatrixXd, Eigen::VectorXd>
  single_measurement_update(double measurement, double measurement_prediction,
                            double measurement_uncertainty, Eigen::VectorXd measurement_jacobian,
                            Eigen::VectorXd x, Eigen::MatrixXd P);

  std::tuple<Eigen::MatrixXd, Eigen::VectorXd> partial_measurement_update(
    Eigen::VectorXd x, Eigen::VectorXd inputs, MeasurementModelFuncRef measurement_model,
    Eigen::VectorXd y, JacobianFuncRef measurement_jacobian, SensorNoiseFuncRef sensor_noise_model,
    Eigen::MatrixXd P, Eigen::VectorXd gammas);

private:
  virtual void estimate(const Input & input, Output & output) override = 0;
};

} // namespace rosplane

#endif // ESTIMATOR_EKF_H
