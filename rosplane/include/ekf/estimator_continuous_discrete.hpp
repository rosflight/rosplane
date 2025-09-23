#ifndef ESTIMATOR_CONTINUOUS_DISCRETE_H // FIXME: redefine the header guards when you rename the ekf.
#define ESTIMATOR_CONTINUOUS_DISCRETE_H

#include <math.h>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include "geomag.h"

#include <cmath>
#include <functional>
#include <tuple>

#include "estimator_ekf.hpp"
#include "estimator_ros.hpp"

namespace rosplane
{

class EstimatorContinuousDiscrete : public EstimatorEKF // FIXME: Rename to something more specific.
{
public:
  EstimatorContinuousDiscrete();
  EstimatorContinuousDiscrete(bool use_params);

protected:
  virtual void estimate(const Input & input, Output & output) override;
private:

  /**
   * @brief The low pass filter alpha value used on the gyro.
   */
  float alpha_gyro_;
  
  /**
   * @brief The low pass filter alpha value used on the barometer.
   */
  float alpha_baro_;

  /**
   * @brief The value of the low pass filtered gyroscope measurement.
   */
  float lpf_gyro_x_;

  /**
   * @brief The value of the low pass filtered gyroscope measurement.
   */
  float lpf_gyro_y_;

  /**
   * @brief The value of the low pass filtered gyroscope measurement.
   */
  float lpf_gyro_z_;

  /**
   * @brief The value of the low pass filtered static pressure sensor (barometer).
   */
  float lpf_static_;

  /**
   * @brief The value of the differential pressure.
   */
  float diff_;

  /**
   * @brief The low pass filter alpha value used on the airspeed.
   */
  float alpha_va_;
  
  /**
   * @brief The lowpass filtered airspeed measurement.
   */
  float lpf_va_;
  
  /**
   * @brief This function calculates the derivatives of the state. This is dictated by the dynamics of
   * the system.
   *
   * @param state The state of the system. 
   * @param inputs The inputs to the estimator. Can be something like IMU measurements.
   */
  Eigen::VectorXf dynamics(const Eigen::VectorXf& state, const Eigen::VectorXf& inputs);

  /**
   * @brief This is a reference to the dynamics function, this is created by the std::bind.
   * This offers a minimum time penalty when passed into a function.
   */
  DynamicModelFuncRef dynamics_model;

  /**
   * @brief Calculates the jacobian of the system dynamics given the current states and inputs.
   *
   * @param state The state of the system.
   * @param inputs The inputs to the estimator, something like IMU measurements.
   */
  Eigen::MatrixXf jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& inputs);

  /**
   * @brief This is a reference to the jacobian function, this is created by the std::bind.
   * This offers a minimum time penalty when passed into a function.
   */
  JacobianFuncRef jacobian_model;

  /**
   * @brief Calculates the jacobian of the inputs to the estimator.
   *
   * @param state The state of the dynamic system.
   * @param inputs Inputs to the estimator.
   */
  Eigen::MatrixXf input_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& inputs);

  /**
   * @brief This is a reference to the input_jacobian function. This incurs minimum time cost
   * when passing into a function.
   */
  JacobianFuncRef input_jacobian_model;

  /**
   * @brief Calculates prediction of the measurements using a model of the sensors.
   *
   * @param state The state of the dynamic system.
   * @param input Inputs to the measurement prediction. Essentially information necessary to the prediction,
   * but is not contained in the state.
   */
  Eigen::VectorXf gnss_measurement_prediction(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  /**
   * @brief This is a reference to the measurement_prediction function. This incurs minimum time cost
   * when passing into a function.
   */
   MeasurementModelFuncRef gnss_measurement_model;
  
  /**
   * @brief Calculates the measurement jacobian for the measurement model.
   *
   * @param state State of the system.
   * @param input Any necessary inputs not included in the state.
   */
  Eigen::MatrixXf gnss_measurement_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& input);

  /**
   * @brief This is a reference to the measurement_jacobian function. This incurs minimum time cost
   * when passing into a function.
   */
  JacobianFuncRef gnss_measurement_jacobian_model;
  
  /**
   * @brief This function returns the GNSS measurement noise.
   */
  Eigen::MatrixXf gnss_measurement_sensor_noise();

  /**
   * @brief Calculates the partial of gravity in the body frame with respect to the Euler angles.
   */
  Eigen::Matrix<float, 3,3> del_R_Theta_T_g_del_Theta(const Eigen::Vector3f& Theta, const double& gravity);

  /**
   * @brief Calculates the partial the inertial velocities with respect to the Euler angles.
   */
  Eigen::Matrix<float, 3,3> del_R_Theta_v_del_Theta(const Eigen::Vector3f& Theta, const Eigen::Vector3f& vels);

  /**
   * @brief Acceleration due to gravity in m/s^2.
   */
  double gravity_ = 9.81;
  
  /**
   * @brief Reference to the function that calculates the sensor noise of the GNSS.
   */
  SensorNoiseFuncRef gnss_measurement_sensor_noise_model;
  
  /**
   * @brief Calculates measurement prediction for the mag.
   *
   * @param state The state of the system.
   * @param input Inputs to the measurement prediction. Essentially information necessary to the prediction,
   * but is not contained in the state.
   */
  Eigen::VectorXf mag_measurement_prediction(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  Eigen::VectorXf tilt_mag_measurement_prediction(const Eigen::VectorXf& state, const Eigen::VectorXf& input);

  /**
   * @brief This is a reference to the mag_measurement_prediction function. This incurs minimum time cost
   * when passing into a function.
   */
  MeasurementModelFuncRef mag_measurement_model;
  MeasurementModelFuncRef tilt_mag_measurement_model;
  
  /**
   * @brief Calculates the jacobian of the measurement model for the magnetometer.
   *
   * @param state State of the system.
   * @param input Any inputs not in the state needed for the system.
   */
  Eigen::MatrixXf mag_measurement_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  Eigen::MatrixXf tilt_mag_measurement_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  
  /**
   * @brief This is a reference to the mag_measurement_jacobian function. This incurs minimum time cost
   * when passing into a function.
   */
  JacobianFuncRef mag_measurement_jacobian_model;
  JacobianFuncRef tilt_mag_measurement_jacobian_model;
  
  /**
   * @brief Calculates the sensor noise of the magnetometer.
   */
  Eigen::MatrixXf mag_measurement_sensor_noise();
  Eigen::MatrixXf tilt_mag_measurement_sensor_noise();
  
  /**
   * @brief Reference to the magnetometer sensor noise calculation.
   */
  SensorNoiseFuncRef mag_measurement_sensor_noise_model;
  SensorNoiseFuncRef tilt_mag_measurement_sensor_noise_model;
  
  /**
   * @brief Calculates measurement prediction for the baro.
   *
   * @param state The state of the system.
   * @param input Inputs to the measurement prediction. Essentially information necessary to the prediction,
   * but is not contained in the state.
   */
  Eigen::VectorXf baro_measurement_prediction(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  /**
   * @brief This is a reference to the baro_measurement_prediction function. This incurs minimum time cost
   * when passing into a function.
   */
  MeasurementModelFuncRef baro_measurement_model;
  
  /**
   * @brief Calculates the jacobian of the measurement model for the barometer.
   *
   * @param state State of the system.
   * @param input Any inputs not in the state needed for the system.
   */
  Eigen::MatrixXf baro_measurement_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  /**
   * @brief This is a reference to the baro_measurement_jacobian function. This incurs minimum time cost
   * when passing into a function.
   */
  JacobianFuncRef baro_measurement_jacobian_model;
  
  /**
   * @brief Calculates the barometer sensor noise.
   */
  Eigen::MatrixXf baro_measurement_sensor_noise();

  void beta_pseudo_measurement_update_step(const Input& input);
  
  /**
   * @brief Reference to the calculation of the barometer sensor noise.
   */
  SensorNoiseFuncRef baro_measurement_sensor_noise_model;
  
  /**
   * @brief Calculates measurement prediction for the wind pseudo measurement.
   *
   * @param state The state of the system.
   * @param input Inputs to the measurement prediction. Essentially information necessary to the prediction,
   * but is not contained in the state.
   */
  Eigen::VectorXf beta_pseudo_measurement_prediction(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  /**
   * @brief This is a reference to the baro_measurement_prediction function. This incurs minimum time cost
   * when passing into a function.
   */
  MeasurementModelFuncRef beta_pseudo_measurement_model;
  
  /**
   * @brief Calculates the jacobian of the measurement model for the barometer.
   *
   * @param state State of the system.
   * @param input Any inputs not in the state needed for the system.
   */
  Eigen::MatrixXf beta_pseudo_measurement_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  /**
   * @brief This is a reference to the baro_measurement_jacobian function. This incurs minimum time cost
   * when passing into a function.
   */
  JacobianFuncRef beta_pseudo_measurement_jacobian_model;
  
  /**
   * @brief Calculates the barometer sensor noise.
   */
  Eigen::MatrixXf beta_pseudo_measurement_sensor_noise();
  
  /**
   * @brief Reference to the calculation of the barometer sensor noise.
   */
  SensorNoiseFuncRef beta_pseudo_measurement_sensor_noise_model;
  
  /**
   * @brief Calculates measurement prediction for the wind pseudo measurement.
   *
   * @param state The state of the system.
   * @param input Inputs to the measurement prediction. Essentially information necessary to the prediction,
   * but is not contained in the state.
   */
  Eigen::VectorXf pseudo_measurement_prediction(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  /**
   * @brief This is a reference to the baro_measurement_prediction function. This incurs minimum time cost
   * when passing into a function.
   */
  MeasurementModelFuncRef pseudo_measurement_model;
  
  /**
   * @brief Calculates the jacobian of the measurement model for the barometer.
   *
   * @param state State of the system.
   * @param input Any inputs not in the state needed for the system.
   */
  Eigen::MatrixXf pseudo_measurement_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  /**
   * @brief This is a reference to the baro_measurement_jacobian function. This incurs minimum time cost
   * when passing into a function.
   */
  JacobianFuncRef pseudo_measurement_jacobian_model;
  
  /**
   * @brief Calculates the barometer sensor noise.
   */
  Eigen::MatrixXf pseudo_measurement_sensor_noise();
  
  /**
   * @brief Reference to the calculation of the barometer sensor noise.
   */
  SensorNoiseFuncRef pseudo_measurement_sensor_noise_model;

  /**
   * @brief The number of states to estimate.
   */
  static constexpr int num_states = 14; // TODO: Should I add course?

  /**
   * @brief The state of the system.
   */
  Eigen::Vector<float, num_states> xhat_;
  /**
   * @brief The covariance of the estimate.
   */
  Eigen::Matrix<float, num_states, num_states> P_;

  /**
   * @brief The process noise for state propagation.
   */
  Eigen::Matrix<float, num_states, num_states> Q_;
  
  /**
   * @brief There are 6 estimator inputs by default. accel_x, accel_y, accel_z, omega_x, omega_y and omega_z.
   */
  static constexpr int num_estimator_inputs = 6;

  /**
   * @brief The process noise from the inputs to the estimator, accelerations (3) and angular velocities (3).
   * The first 3 rows are for the accels, and the second 3 for the angular velocities.
   */
  Eigen::Matrix<float, num_estimator_inputs, num_estimator_inputs> Q_inputs_;

  /**
   * @brief There are 6 gnss measurements by default. Lat, lon, alt, v_n, v_e and v_d.
   */
  static constexpr int num_gnss_measurements = 5;

  /**
   * @brief The sensor noises for the GNSS measurements. The first three rows are for the positional measurements.
   * The last three rows are for the velocity measurements.
   */
  Eigen::Matrix<float, num_gnss_measurements, num_gnss_measurements> R_gnss_;
  
  /**
   * @brief There are 3 mag measurements by default. m_x, m_y and m_z.
   */
  static constexpr int num_mag_measurements = 3;

  /**
   * @brief The sensor noises for the magnetometer.
   */
  Eigen::Matrix<float, num_mag_measurements, num_mag_measurements> R_mag_;
  
  /**
   * @brief There is one barometer measurement by default. P (pressure).
   */
  static constexpr int num_baro_measurements = 1;
  
  /**
   * @brief The sensor noises for the barometer.
   */
  Eigen::Matrix<float, num_baro_measurements, num_baro_measurements> R_baro_;

  /**
   * @brief There is are 2 pseudo measurements for wind.
   */
  static constexpr int num_pseudo_measurements = 2;
  
  /**
   * @brief The sensor noises for the pseudo measurement.
   */
  Eigen::Matrix<float, num_pseudo_measurements, num_pseudo_measurements> R_pseudo_;
  
  /**
   * @brief There is one differential pressure measurement P (pressure).
   */
  static constexpr int num_diff_measurements = 1;
  
  /**
   * @brief The sensor noises for the barometer.
   */
  Eigen::Matrix<float, num_diff_measurements, num_diff_measurements> R_diff_;

  /**
   * @brief Calculates measurement prediction for the wind pseudo measurement.
   *
   * @param state The state of the system.
   * @param input Inputs to the measurement prediction. Essentially information necessary to the prediction,
   * but is not contained in the state.
   */
  Eigen::VectorXf diff_pressure_measurement_prediction(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  /**
   * @brief This is a reference to the baro_measurement_prediction function. This incurs minimum time cost
   * when passing into a function.
   */
  MeasurementModelFuncRef diff_pressure_measurement_model;
  
  /**
   * @brief Calculates the jacobian of the measurement model for the barometer.
   *
   * @param state State of the system.
   * @param input Any inputs not in the state needed for the system.
   */
  Eigen::MatrixXf diff_pressure_measurement_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& input);
  /**
   * @brief This is a reference to the baro_measurement_jacobian function. This incurs minimum time cost
   * when passing into a function.
   */
  JacobianFuncRef diff_pressure_measurement_jacobian_model;
  
  /**
   * @brief Calculates the barometer sensor noise.
   */
  Eigen::MatrixXf diff_pressure_measurement_sensor_noise();
  
  /**
   * @brief Reference to the calculation of the barometer sensor noise.
   */
  SensorNoiseFuncRef diff_pressure_measurement_sensor_noise_model;
  
  /**
   * @brief The calculated inclination of the magnetic field at the current location.
   */
  double inclination_;

  /**
   * @brief The calculated declination of the magnetic field at the current location.
   */
  double declination_;

  /**
   * @brief Flag to indicate if the state has been Initialized.
   */
  bool state_init_;

  /**
   * @brief Initializes the state based on the current input.
   */
  void init_state(const Input & input);

  /**
   * @brief Run the prediction step of the estimation algorithm.
   */
  void prediction_step(const Input& input);

  /**
   * @brief Run the magnetometer measurement update.
   */
  void mag_measurement_update_step(const Input& input);

/**
   * @brief Run the differential pressure measurement update.
   */
  void diff_measurement_update_step(const Input& input);

  /**
   * @brief Run the barometer measurement update.
   */
  void baro_measurement_update_step(const Input& input);

  /**
   * @brief Run the GNSS measurement update.
   */
  void gnss_measurement_update_step(const Input& input);

  /**
   * @brief Calculates the properties of teh magnetic field at this particular lat lon.
   */
  void calc_mag_field_properties(const Input& input);

  /**
   * @brief Calculates the inertial magnetic field.
   */
  Eigen::Vector3f calculate_inertial_magnetic_field(const float& declination, const float& inclination);
  
  /**
   * @brief Calculates the body to inertial rotation matrix.
   */
  Eigen::Matrix3f R(const Eigen::Vector3f& Theta);
  
  /**
   * @brief Calculates the matrix that integrates gyro measurements into Euler angles.
   */
  Eigen::Matrix3f S(const Eigen::Vector3f& Theta);
  
  /**
   * @brief Calculates the partial of the gyro integration matrix with respect to the Euler angles.
   */
  Eigen::Matrix3f del_S_Theta_del_Theta(const Eigen::Vector3f& Theta, const Eigen::Vector3f& biases, const Eigen::Vector3f& gyro);
  
  /**
   * @brief Calculates the partial of body magnetic field measurements with respect to the Euler angles.
   */
  Eigen::Matrix3f del_R_Theta_T_y_mag_del_Theta(const Eigen::Vector3f& Theta, const Eigen::Vector3f& mag);

  /**
   * @brief This function binds references to the functions used in the ekf.
   */
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
  
  /*
   * @brief Indicates if a parameter in the estimator was changed.
  */
  bool is_parameter_changed();

  /*
   * @brief Updates the process noises and measurement noises.
  */
  void update_estimation_params();

  /**
   * @brief Initializes the process noise matrices with the ROS2 parameters
   */
  void initialize_process_noises();

  /**
   * @brief Initializes the state covariance matrix with the ROS2 parameters
   */
  void initialize_state_covariances();

  /*
   * @brief Checks if the estimate is outside of acceptable values and resets it.
  */
  void check_estimate(const Input& input);
}; 

} // namespace roscopter

#endif // ESTIMATOR_CONTINUOUS_DISCRETE_H
