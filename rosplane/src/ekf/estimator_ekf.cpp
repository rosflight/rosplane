#include <functional>
#include <rclcpp/logging.hpp>
#include <tuple>

#include "ekf/estimator_ekf.hpp"
#include "ekf/estimator_ros.hpp"

namespace rosplane
{

EstimatorEKF::EstimatorEKF() : EstimatorROS()
{
  params_.declare_int("num_propagation_steps", 1);
  params_.set_parameters();
}

std::tuple<Eigen::MatrixXf, Eigen::VectorXf> EstimatorEKF::kalman_update(Eigen::VectorXf x, // TODO: make x and P const.
                                                                         Eigen::VectorXf h,
                                                                         Eigen::VectorXf y,
                                                                         Eigen::MatrixXf C,
                                                                         Eigen::MatrixXf R,
                                                                         Eigen::MatrixXf S,
                                                                         Eigen::MatrixXf P)

{
  // Find the Kalman gain.
  Eigen::MatrixXf L = P * C.transpose() * S.inverse();
  // Use a temp to increase readablility.
  Eigen::MatrixXf temp = Eigen::MatrixXf::Identity(x.size(), x.size()) - L * C;
  
  // Adjust the covariance with new information.
  // This uses Joseph's Stabilized form of the covariance update.
  // This is numerically stable and results in P always being positive definite.
  P = temp * P * temp.transpose() + L * R * L.transpose();
  // Use Kalman gain to optimally adjust estimate.

  x = x + L * (y - h);

  if ((L * (y-h)).any() > 100.0) {
    RCLCPP_INFO_STREAM(this->get_logger(), "CORRECTION LARGE");
  }

  std::tuple<Eigen::MatrixXf, Eigen::VectorXf> result(P, x);
  
  return result;
}

std::tuple<Eigen::MatrixXf, Eigen::VectorXf> EstimatorEKF::measurement_update(Eigen::VectorXf x,
                                                                Eigen::VectorXf inputs,
                                                                MeasurementModelFuncRef measurement_model,
                                                                Eigen::VectorXf y,
                                                                JacobianFuncRef measurement_jacobian,
                                                                SensorNoiseFuncRef sensor_noise_model,
                                                                Eigen::MatrixXf P)
{
  
  Eigen::VectorXf h = measurement_model(x, inputs);
  Eigen::MatrixXf C = measurement_jacobian(x, inputs);
  Eigen::MatrixXf R = sensor_noise_model();
  
  // Find the innovation covariance and it's inverse to find the Kalman gain.
  Eigen::MatrixXf S = (R + C * P * C.transpose());
  
  /*RCLCPP_INFO_STREAM(this->get_logger(), "P: " << P);*/
  /*RCLCPP_INFO_STREAM(this->get_logger(), "S: " << S);*/
  /*RCLCPP_INFO_STREAM(this->get_logger(), "R: " << R);*/
  
  return kalman_update(x, h, y, C, R, S, P);
}

std::tuple<Eigen::MatrixXf, Eigen::VectorXf> EstimatorEKF::propagate_model(Eigen::VectorXf x,
                                                             DynamicModelFuncRef dynamic_model,
                                                             JacobianFuncRef jacobian,
                                                             Eigen::VectorXf inputs,
                                                             JacobianFuncRef input_jacobian,
                                                             Eigen::MatrixXf P,
                                                             Eigen::MatrixXf Q,
                                                             Eigen::MatrixXf Q_g,
                                                             float Ts)
{

  int N = params_.get_int("num_propagation_steps");

  for (int _ = 0; _ < N; _++)
  {

    Eigen::VectorXf f = dynamic_model(x, inputs);
    // Propagate model by a step.
    x += f * (Ts/N);

    Eigen::MatrixXf A = jacobian(x, inputs);
    
    // Find the second order approx of the matrix exponential.
    Eigen::MatrixXf A_d = Eigen::MatrixXf::Identity(A.rows(), A.cols()) + Ts / N * A
      + pow(Ts / N, 2) / 2.0 * A * A;

    Eigen::MatrixXf G = input_jacobian(x, inputs);
    
    // Propagate the covariance.
    P = A_d * P * A_d.transpose() + (Q + G * Q_g * G.transpose()) * pow(Ts / N, 2);
    
  }

  std::tuple<Eigen::MatrixXf, Eigen::VectorXf> result(P, x);
  
  return result;
}

std::tuple<Eigen::MatrixXf, Eigen::VectorXf> EstimatorEKF::single_measurement_update(float measurement, float measurement_prediction,
                                                                                     float measurement_variance, Eigen::VectorXf measurement_jacobian,
                                                                                     Eigen::VectorXf x, Eigen::MatrixXf P)
{
  Eigen::MatrixXf I(x.size(),x.size());
  I = Eigen::MatrixXf::Identity(x.size(), x.size());
  Eigen::VectorXf L = (P * measurement_jacobian) / (measurement_variance + (measurement_jacobian.transpose() * P * measurement_jacobian));
  P = (I - L * measurement_jacobian.transpose()) * P;
  x = x + L * (measurement - measurement_prediction);

  std::tuple<Eigen::MatrixXf, Eigen::VectorXf> result(P,x);
  return result;
}

std::tuple<Eigen::MatrixXf, Eigen::VectorXf> EstimatorEKF::partial_measurement_update(Eigen::VectorXf x,
                                                                Eigen::VectorXf inputs,
                                                                MeasurementModelFuncRef measurement_model,
                                                                Eigen::VectorXf y,
                                                                JacobianFuncRef measurement_jacobian,
                                                                SensorNoiseFuncRef sensor_noise_model,
                                                                Eigen::MatrixXf P,
                                                                Eigen::VectorXf gammas)
{

  // See Partial-Update Schmidt-Kalman Filter, Kevin Brink, 2017 Journal of Guidance, Control and Dynamics.
  // Specifcially Equations 68-69, the algorithm described in Section V subsection B.
  
  Eigen::VectorXf h = measurement_model(x, inputs);
  Eigen::MatrixXf C = measurement_jacobian(x, inputs);
  Eigen::MatrixXf R = sensor_noise_model();
  
  // Find the S_inv to find the Kalman gain.
  Eigen::MatrixXf S = (R + C * P * C.transpose());
  
  Eigen::MatrixXf P_update;
  Eigen::VectorXf x_update;

  std::tie(P_update, x_update) = kalman_update(x, h, y, C, R, S, P);

  Eigen::VectorXf ones = Eigen::VectorXf::Ones(x.size());

  x = (gammas.array()*x.array()).matrix() + ((ones - gammas).array()*x_update.array()).matrix();

  auto gamma_outer_product = gammas*gammas.transpose();
  Eigen::MatrixXf ones_matrix = Eigen::MatrixXf::Constant(x.size(), x.size(), 1);
  
  P = (gamma_outer_product.array() * P.array()).matrix() + ((ones_matrix-gamma_outer_product).array()*P_update.array()).matrix();

  std::tuple<Eigen::MatrixXf, Eigen::VectorXf> result(P, x);
  
  return result;
}

} // end nampspace.
