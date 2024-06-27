#include "estimator_ekf.hpp"
#include "estimator_ros.hpp"
#include <tuple>

namespace rosplane
{

estimator_ekf::estimator_ekf() : estimator_ros()
{}
  
std::tuple<Eigen::MatrixXf, Eigen::VectorXf> estimator_ekf::measurement_update(Eigen::VectorXf x,
                                                                Eigen::VectorXf u,
                                                                Eigen::VectorXf h,
                                                                Eigen::VectorXf y,
                                                                Eigen::MatrixXf R,
                                                                Eigen::MatrixXf C,
                                                                Eigen::MatrixXf P,
                                                                float Ts)
{

  check_measurment_update_input(x, u, h, y, R, C, P, Ts);
  
  // Find the S_inv to find the Kalman gain.
  Eigen::MatrixXf S_inv = (R + C * P * C.transpose()).inverse();
  // Find the Kalman gain.
  Eigen::MatrixXf L = P * C.transpose() * S_inv;
  // Use a temp to increase readablility.
  Eigen::MatrixXf temp = Eigen::MatrixXf::Identity(2, 2) - L * C;
  
  // Adjust the covariance with new information.
  P = temp * P * temp.transpose() + L * R * L.transpose();
  // Use Kalman gain to optimally adjust estimate.
  x = x + L * (y - h);

  std::tuple<Eigen::MatrixXf, Eigen::VectorXf> result(P, x);
  
  return result;
}

std::tuple<Eigen::MatrixXf, Eigen::VectorXf> estimator_ekf::propagate_model(Eigen::VectorXf x,
                                                             Eigen::VectorXf f,
                                                             Eigen::MatrixXf A,
                                                             Eigen::MatrixXf P,
                                                             Eigen::MatrixXf G,
                                                             Eigen::MatrixXf Q_g,
                                                             Eigen::MatrixXf Q,
                                                             float Ts)
{

  check_propagate_model_input(x, f, A, P, G, Q_g, Q, Ts);

  int N = params.get_int("num_propagation_steps");

  for (int _ = 0; _ < N; _++)
  {
    // Propagate model by a step.
    x += f * (Ts/N);
    
    // Find the second order approx of the matrix exponential.
    Eigen::MatrixXf A_d = Eigen::MatrixXf::Identity(A.rows(), A.cols()) + Ts / N * A
      + pow(Ts / N, 2) / 2.0 * A * A;

    // Propagate the covariance.
    P = A_d * P * A_d.transpose() + (Q + G * Q_g * G.transpose() * pow(Ts / N, 2));
    
  }

  std::tuple<Eigen::MatrixXf, Eigen::VectorXf> result(P, x);
  
  return result;
}

void estimator_ekf::check_measurment_update_input(Eigen::VectorXf x,
                                   Eigen::VectorXf u,
                                   Eigen::VectorXf h,
                                   Eigen::VectorXf y,
                                   Eigen::MatrixXf R,
                                   Eigen::MatrixXf C,
                                   Eigen::MatrixXf P,
                                   float Ts)
{
  assert(P.cols() == C.cols() && "Covariance (P) and measurement Jacobian (C) are incompatible sizes.");
  assert(R.rows() == C.rows() && "Measurement noise matrix (R) and measurement Jacobian (C) are incompatible sizes.");
  assert(y.size() == h.size() && "Measurement model (h) and measurement vector (y) are incompatible sizes.");
}

void estimator_ekf::check_propagate_model_input(Eigen::VectorXf x,
                                                  Eigen::VectorXf f,
                                                  Eigen::MatrixXf A,
                                                  Eigen::MatrixXf P,
                                                  Eigen::MatrixXf G,
                                                  Eigen::MatrixXf Q_g,
                                                  Eigen::MatrixXf Q,
                                                  float Ts)
{
  assert(f.size() == x.size() && "Dynamic model (f) and state vector (x) have incompatible sizes.");
  assert(A.rows() == x.size() && "Jacobian (A) and state vector (x) have incompatible sizes.");
  assert(A.rows() == A.cols() && "Jacobian (A) is not square.");
  assert(A.rows() == P.rows() && "Jacobian (A) and covariance (P) have incompatible sizes.");
  assert(Q.rows() == G.rows() && "Process noise matrix (Q) and input Jacobian (G) have incompatible sizes.");
  assert(G.cols() == Q_g.rows() && "Input noise matrix (Q_g) and input Jacobian (G) have incompatible sizes.");
}

} // end nampspace.
