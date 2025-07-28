#ifndef ESTIMATOR_CONTINUOUS_DISCRETE_H
#define ESTIMATOR_CONTINUOUS_DISCRETE_H

#include <math.h>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include "estimator_ekf.hpp"
#include "estimator_ros.hpp"

namespace rosplane
{

class EstimatorContinuousDiscrete : public EstimatorEKF
{
public:
  EstimatorContinuousDiscrete();
  EstimatorContinuousDiscrete(bool use_params);

private:
  virtual void estimate(const Input & input, Output & output);

  double lpf_a_;
  float alpha_;
  float alpha1_;
  int N_;

  float lpf_gyro_x_;
  float lpf_gyro_y_;
  float lpf_gyro_z_;
  float lpf_static_;
  float lpf_diff_;
  float lpf_accel_x_;
  float lpf_accel_y_;
  float lpf_accel_z_;

  float phat_;
  float qhat_;
  float rhat_;
  float Vwhat_;
  float phihat_;
  float thetahat_;
  float psihat_; // TODO: link to an inital condiditons param

  Eigen::VectorXf attitude_dynamics(const Eigen::VectorXf & state,
                                    const Eigen::VectorXf & measurements);
  std::function<Eigen::VectorXf(const Eigen::VectorXf, const Eigen::VectorXf)>
    attitude_dynamics_model;

  Eigen::MatrixXf attitude_jacobian(const Eigen::VectorXf & state,
                                    const Eigen::VectorXf & angular_rates);
  std::function<Eigen::MatrixXf(const Eigen::VectorXf &, const Eigen::VectorXf &)>
    attitude_jacobian_model;

  Eigen::MatrixXf attitude_input_jacobian(const Eigen::VectorXf & state,
                                          const Eigen::VectorXf & angular_rates);
  std::function<Eigen::MatrixXf(const Eigen::VectorXf &, const Eigen::VectorXf &)>
    attitude_input_jacobian_model;

  Eigen::VectorXf attitude_measurement_prediction(const Eigen::VectorXf & state,
                                                  const Eigen::VectorXf & inputs);
  std::function<Eigen::VectorXf(const Eigen::VectorXf, const Eigen::VectorXf)>
    attitude_measurement_model;

  Eigen::MatrixXf attitude_measurement_jacobian(const Eigen::VectorXf & state,
                                                const Eigen::VectorXf & inputs);
  std::function<Eigen::MatrixXf(const Eigen::VectorXf, const Eigen::VectorXf)>
    attitude_measurement_jacobian_model;

  Eigen::VectorXf position_dynamics(const Eigen::VectorXf & state,
                                    const Eigen::VectorXf & measurements);
  std::function<Eigen::VectorXf(const Eigen::VectorXf, const Eigen::VectorXf)>
    position_dynamics_model;

  Eigen::MatrixXf position_jacobian(const Eigen::VectorXf & state,
                                    const Eigen::VectorXf & measurements);
  std::function<Eigen::MatrixXf(const Eigen::VectorXf &, const Eigen::VectorXf &)>
    position_jacobian_model;

  Eigen::MatrixXf position_input_jacobian(const Eigen::VectorXf & state,
                                          const Eigen::VectorXf & inputs);
  std::function<Eigen::MatrixXf(const Eigen::VectorXf &, const Eigen::VectorXf &)>
    position_input_jacobian_model;

  Eigen::VectorXf position_measurement_prediction(const Eigen::VectorXf & state,
                                                  const Eigen::VectorXf & input);
  std::function<Eigen::VectorXf(const Eigen::VectorXf, const Eigen::VectorXf)>
    position_measurement_model;

  Eigen::MatrixXf position_measurement_jacobian(const Eigen::VectorXf & state,
                                                const Eigen::VectorXf & input);
  std::function<Eigen::MatrixXf(const Eigen::VectorXf, const Eigen::VectorXf)>
    position_measurement_jacobian_model;

  Eigen::Vector2f xhat_a_; // 2
  Eigen::Matrix2f P_a_;    // 2x2

  Eigen::VectorXf xhat_p_; // 7
  Eigen::MatrixXf P_p_;    // 7x7

  Eigen::Matrix2f Q_a_; // 2x2
  Eigen::Matrix3f Q_g_;
  Eigen::Matrix3f R_accel_;

  Eigen::MatrixXf Q_p_; // 7x7
  Eigen::MatrixXf R_p_; // 6x6
  Eigen::VectorXf f_p_; // 7
  Eigen::MatrixXf A_p_; // 7x7
  float h_p_;
  Eigen::VectorXf C_p_; // 7
  Eigen::VectorXf L_p_; // 7

  // TODO: not used
  float gate_threshold_ = 9.21; // chi2(q = .01, df = 2)

  void check_xhat_a();

  void bind_functions();

  /**
   * @brief This declares each parameter as a parameter so that the ROS2 parameter system can recognize each parameter.
   * It also sets the default parameter, which will then be overridden by a launch script.
   */
  void declare_parameters();

  /**
   * @brief Initializes some variables that depend on ROS2 parameters
  */
  void update_measurement_model_parameters();

  /**
   * @brief Initializes the covariance matrices and process noise matrices with the ROS2 parameters
   */
  void initialize_uncertainties();

  /**
   * @brief Initializes the state covariance matrix with the ROS2 parameters
   */
  void initialize_state_covariances();
};

} // namespace rosplane

#endif // ESTIMATOR_CONTINUOUS_DISCRETE_H
