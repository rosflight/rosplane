#include <functional>
#include <tuple>

#include <rclcpp/logging.hpp>

#include "estimator_ekf.hpp"
#include "estimator_ros.hpp"

namespace rosplane
{

EstimatorEKF::EstimatorEKF()
    : EstimatorROS()
{}

std::tuple<Eigen::MatrixXf, Eigen::VectorXf> EstimatorEKF::measurement_update(
  Eigen::VectorXf x, Eigen::VectorXf inputs,
  std::function<Eigen::VectorXf(const Eigen::VectorXf, const Eigen::VectorXf)> measurement_model,
  Eigen::VectorXf y,
  std::function<Eigen::MatrixXf(const Eigen::VectorXf, const Eigen::VectorXf)> measurement_jacobian,
  Eigen::MatrixXf R, Eigen::MatrixXf P)
{

  Eigen::VectorXf h = measurement_model(x, inputs);
  Eigen::MatrixXf C = measurement_jacobian(x, inputs);

  // Find the S_inv to find the Kalman gain.
  Eigen::MatrixXf S_inv = (R + C * P * C.transpose()).inverse();
  // Find the Kalman gain.
  Eigen::MatrixXf L = P * C.transpose() * S_inv;
  // Use a temp to increase readablility.
  Eigen::MatrixXf temp = Eigen::MatrixXf::Identity(x.size(), x.size()) - L * C;

  // Adjust the covariance with new information.
  P = temp * P * temp.transpose() + L * R * L.transpose();
  // Use Kalman gain to optimally adjust estimate.
  x = x + L * (y - h);

  std::tuple<Eigen::MatrixXf, Eigen::VectorXf> result(P, x);

  return result;
}

std::tuple<Eigen::MatrixXf, Eigen::VectorXf> EstimatorEKF::propagate_model(
  Eigen::VectorXf x,
  std::function<Eigen::VectorXf(const Eigen::VectorXf &, const Eigen::VectorXf &)> dynamic_model,
  std::function<Eigen::MatrixXf(const Eigen::VectorXf &, const Eigen::VectorXf &)> jacobian,
  Eigen::VectorXf inputs,
  std::function<Eigen::MatrixXf(const Eigen::VectorXf &, const Eigen::VectorXf &)> input_jacobian,
  Eigen::MatrixXf P, Eigen::MatrixXf Q, Eigen::MatrixXf Q_g, float Ts)
{

  int N =
    params_.get_int("num_propagation_steps"); // TODO: Declare this parameter in the ekf class.

  for (int _ = 0; _ < N; _++) {

    Eigen::VectorXf f = dynamic_model(x, inputs);
    // Propagate model by a step.
    x += f * (Ts / N);

    Eigen::MatrixXf A = jacobian(x, inputs);

    // Find the second order approx of the matrix exponential.
    Eigen::MatrixXf A_d =
      Eigen::MatrixXf::Identity(A.rows(), A.cols()) + Ts / N * A + pow(Ts / N, 2) / 2.0 * A * A;

    Eigen::MatrixXf G = input_jacobian(x, inputs);

    // Propagate the covariance.
    P = A_d * P * A_d.transpose() + (Q + G * Q_g * G.transpose()) * pow(Ts / N, 2);
  }

  std::tuple<Eigen::MatrixXf, Eigen::VectorXf> result(P, x);

  return result;
}

std::tuple<Eigen::MatrixXf, Eigen::VectorXf> EstimatorEKF::single_measurement_update(
  float measurement, float measurement_prediction, float measurement_variance,
  Eigen::VectorXf measurement_jacobian, Eigen::VectorXf x, Eigen::MatrixXf P)
{
  Eigen::MatrixXf I(x.size(), x.size());
  I = Eigen::MatrixXf::Identity(x.size(), x.size());
  Eigen::VectorXf L = (P * measurement_jacobian)
    / (measurement_variance + (measurement_jacobian.transpose() * P * measurement_jacobian));
  P = (I - L * measurement_jacobian.transpose()) * P;
  x = x + L * (measurement - measurement_prediction);

  std::tuple<Eigen::MatrixXf, Eigen::VectorXf> result(P, x);
  return result;
}

} // namespace rosplane
