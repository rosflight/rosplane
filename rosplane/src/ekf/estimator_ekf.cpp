#include "ekf/estimator_ekf.hpp"

namespace rosplane
{

EstimatorEKF::EstimatorEKF() : EstimatorROS()
{
  params_.declare_int("num_propagation_steps", 1);
  params_.set_parameters();
}

std::tuple<Eigen::MatrixXd, Eigen::VectorXd> EstimatorEKF::kalman_update(Eigen::VectorXd x,
                                                                         Eigen::VectorXd h,
                                                                         Eigen::VectorXd y,
                                                                         Eigen::MatrixXd C,
                                                                         Eigen::MatrixXd R,
                                                                         Eigen::MatrixXd S,
                                                                         Eigen::MatrixXd P)

{
  // Find the Kalman gain.
  Eigen::MatrixXd L = P * C.transpose() * S.inverse();
  // Use a temp to increase readablility.
  Eigen::MatrixXd temp = Eigen::MatrixXd::Identity(x.size(), x.size()) - L * C;
  
  // Adjust the covariance with new information.
  // This uses Joseph's Stabilized form of the covariance update.
  // This is numerically stable and results in P always being positive definite.
  P = temp * P * temp.transpose() + L * R * L.transpose();
  // Use Kalman gain to optimally adjust estimate.

  x = x + L * (y - h);

  std::tuple<Eigen::MatrixXd, Eigen::VectorXd> result(P, x);
  
  return result;
}

std::tuple<Eigen::MatrixXd, Eigen::VectorXd> EstimatorEKF::measurement_update(Eigen::VectorXd x,
                                                                Eigen::VectorXd inputs,
                                                                MeasurementModelFuncRef measurement_model,
                                                                Eigen::VectorXd y,
                                                                JacobianFuncRef measurement_jacobian,
                                                                SensorNoiseFuncRef sensor_noise_model,
                                                                Eigen::MatrixXd P)
{
  
  Eigen::VectorXd h = measurement_model(x, inputs);
  Eigen::MatrixXd C = measurement_jacobian(x, inputs);
  Eigen::MatrixXd R = sensor_noise_model(x, inputs);
  
  // Find the innovation covariance and it's inverse to find the Kalman gain.
  Eigen::MatrixXd S = (R + C * P * C.transpose());
  
  return kalman_update(x, h, y, C, R, S, P);
}

std::tuple<Eigen::MatrixXd, Eigen::VectorXd> EstimatorEKF::propagate_model(Eigen::VectorXd x,
                                                             DynamicModelFuncRef dynamic_model,
                                                             JacobianFuncRef jacobian,
                                                             Eigen::VectorXd inputs,
                                                             JacobianFuncRef input_jacobian,
                                                             Eigen::MatrixXd P,
                                                             Eigen::MatrixXd Q,
                                                             Eigen::MatrixXd Q_g,
                                                             double Ts)
{

  int N = params_.get_int("num_propagation_steps");

  for (int _ = 0; _ < N; _++)
  {

    Eigen::VectorXd f = dynamic_model(x, inputs);
    // Propagate model by a step.
    x += f * (Ts/N);

    Eigen::MatrixXd A = jacobian(x, inputs);
    
    // Find the second order approx of the matrix exponential.
    Eigen::MatrixXd A_d = Eigen::MatrixXd::Identity(A.rows(), A.cols()) + Ts / N * A
      + pow(Ts / N, 2) / 2.0 * A * A;

    Eigen::MatrixXd G = input_jacobian(x, inputs);
    
    // Propagate the covariance.
    P = A_d * P * A_d.transpose() + (Q + G * Q_g * G.transpose()) * pow(Ts / N, 2);
    
  }

  std::tuple<Eigen::MatrixXd, Eigen::VectorXd> result(P, x);
  
  return result;
}

std::tuple<Eigen::MatrixXd, Eigen::VectorXd> EstimatorEKF::single_measurement_update(double measurement, double measurement_prediction,
                                                                                     double measurement_variance, Eigen::VectorXd measurement_jacobian,
                                                                                     Eigen::VectorXd x, Eigen::MatrixXd P)
{
  Eigen::MatrixXd I(x.size(),x.size());
  I = Eigen::MatrixXd::Identity(x.size(), x.size());
  Eigen::VectorXd L = (P * measurement_jacobian) / (measurement_variance + (measurement_jacobian.transpose() * P * measurement_jacobian));
  P = (I - L * measurement_jacobian.transpose()) * P;
  x = x + L * (measurement - measurement_prediction);

  std::tuple<Eigen::MatrixXd, Eigen::VectorXd> result(P,x);
  return result;
}

std::tuple<Eigen::MatrixXd, Eigen::VectorXd> EstimatorEKF::partial_measurement_update(Eigen::VectorXd x,
                                                                Eigen::VectorXd inputs,
                                                                MeasurementModelFuncRef measurement_model,
                                                                Eigen::VectorXd y,
                                                                JacobianFuncRef measurement_jacobian,
                                                                SensorNoiseFuncRef sensor_noise_model,
                                                                Eigen::MatrixXd P,
                                                                Eigen::VectorXd gammas)
{

  // See Partial-Update Schmidt-Kalman Filter, Kevin Brink, 2017 Journal of Guidance, Control and Dynamics.
  // Specifcially Equations 68-69, the algorithm described in Section V subsection B.
  
  Eigen::VectorXd h = measurement_model(x, inputs);
  Eigen::MatrixXd C = measurement_jacobian(x, inputs);
  Eigen::MatrixXd R = sensor_noise_model(x, inputs);
  
  // Find the S_inv to find the Kalman gain.
  Eigen::MatrixXd S = (R + C * P * C.transpose());
  
  Eigen::MatrixXd P_update;
  Eigen::VectorXd x_update;

  std::tie(P_update, x_update) = kalman_update(x, h, y, C, R, S, P);

  Eigen::VectorXd ones = Eigen::VectorXd::Ones(x.size());

  x = (gammas.array()*x.array()).matrix() + ((ones - gammas).array()*x_update.array()).matrix();

  auto gamma_outer_product = gammas*gammas.transpose();
  Eigen::MatrixXd ones_matrix = Eigen::MatrixXd::Constant(x.size(), x.size(), 1);
  
  P = (gamma_outer_product.array() * P.array()).matrix() + ((ones_matrix-gamma_outer_product).array()*P_update.array()).matrix();

  std::tuple<Eigen::MatrixXd, Eigen::VectorXd> result(P, x);
  
  return result;
}

} // end nampspace.
