#include <functional>
#include <tuple>

#include "estimator_continuous_discrete.hpp"
#include "estimator_ros.hpp"

namespace rosplane
{

float radians(float degrees) { return M_PI * degrees / 180.0; }

double wrap_within_180(double fixed_heading, double wrapped_heading)
{
  // wrapped_heading - number_of_times_to_wrap * 2pi
  return wrapped_heading - floor((wrapped_heading - fixed_heading) / (2 * M_PI) + 0.5) * 2 * M_PI;
}

EstimatorContinuousDiscrete::EstimatorContinuousDiscrete()
    : EstimatorEKF()
    , xhat_a_(Eigen::Vector2f::Zero())
    , P_a_(Eigen::Matrix2f::Identity())
    , xhat_p_(Eigen::VectorXf::Zero(7))
    , P_p_(Eigen::MatrixXf::Identity(7, 7))
    , Q_a_(Eigen::Matrix2f::Identity())
    , Q_g_(Eigen::Matrix3f::Identity())
    , R_accel_(Eigen::Matrix3f::Identity())
    , Q_p_(Eigen::MatrixXf::Identity(7, 7))
    , R_p_(Eigen::MatrixXf::Zero(7, 7))
    , f_p_(7)
    , A_p_(7, 7)
    , C_p_(7)
    , L_p_(7)
{

  bind_functions(); // TODO: Document what the _models are.

  phat_ = 0;
  qhat_ = 0;
  rhat_ = 0;
  phihat_ = 0;
  thetahat_ = 0;
  psihat_ = 0;
  Vwhat_ = 0;

  lpf_static_ = 0.0;
  lpf_diff_ = 0.0;

  alpha_ = 0.0f;

  // Declare and set parameters with the ROS2 system
  declare_parameters();
  params_.set_parameters();

  // Initialize covariance matrices from ROS2 parameters
  initialize_uncertainties();

  // Inits R matrix and alpha values with ROS2 parameters
  update_measurement_model_parameters();

  N_ = params_.get_int("num_propagation_steps");
}

EstimatorContinuousDiscrete::EstimatorContinuousDiscrete(bool use_params)
    : EstimatorContinuousDiscrete()
{
  double init_lat = params_.get_double("init_lat");
  double init_long = params_.get_double("init_lon");
  double init_alt = params_.get_double("init_alt");
  double init_static = params_.get_double("baro_calibration_val");

  RCLCPP_INFO_STREAM(this->get_logger(), "Using seeded estimator values.");
  RCLCPP_INFO_STREAM(this->get_logger(), "Seeded initial latitude: " << init_lat);
  RCLCPP_INFO_STREAM(this->get_logger(), "Seeded initial longitude: " << init_long);
  RCLCPP_INFO_STREAM(this->get_logger(), "Seeded initial altitude: " << init_alt);
  RCLCPP_INFO_STREAM(this->get_logger(), "Seeded barometer calibration value: " << init_static);

  gps_init_ = true;
  init_lat_ = init_lat;
  init_lon_ = init_long;
  init_alt_ = init_alt;

  baro_init_ = true;
  init_static_ = init_static;
}

void EstimatorContinuousDiscrete::initialize_state_covariances()
{
  double pos_n_initial_cov = params_.get_double("pos_n_initial_cov");
  double pos_e_initial_cov = params_.get_double("pos_e_initial_cov");
  double vg_initial_cov = params_.get_double("vg_initial_cov");
  double chi_initial_cov = params_.get_double("chi_initial_cov");
  double wind_n_initial_cov = params_.get_double("wind_n_initial_cov");
  double wind_e_initial_cov = params_.get_double("wind_e_initial_cov");
  double psi_initial_cov = params_.get_double("psi_initial_cov");

  P_p_ = Eigen::MatrixXf::Identity(7, 7);
  P_p_(0, 0) = pos_n_initial_cov;
  P_p_(1, 1) = pos_e_initial_cov;
  P_p_(2, 2) = vg_initial_cov;
  P_p_(3, 3) = radians(chi_initial_cov);
  P_p_(4, 4) = wind_n_initial_cov;
  P_p_(5, 5) = wind_e_initial_cov;
  P_p_(6, 6) = radians(psi_initial_cov);
}

void EstimatorContinuousDiscrete::initialize_uncertainties()
{
  double roll_process_noise = params_.get_double("roll_process_noise");
  double pitch_process_noise = params_.get_double("pitch_process_noise");
  double gyro_process_noise = params_.get_double("gyro_process_noise");
  double position_process_noise = params_.get_double("pos_process_noise");
  double attitude_initial_cov = params_.get_double("attitude_initial_cov");

  P_a_ *= powf(radians(attitude_initial_cov), 2);

  Q_a_(0, 0) = roll_process_noise;
  Q_a_(1, 1) = pitch_process_noise;

  Q_g_ *= pow(radians(gyro_process_noise), 2);

  Q_p_ *= position_process_noise;

  initialize_state_covariances();
}

void EstimatorContinuousDiscrete::update_measurement_model_parameters()
{
  // For readability, declare the parameters used in the function here
  double sigma_n_gps = params_.get_double("sigma_n_gps");
  double sigma_e_gps = params_.get_double("sigma_e_gps");
  double sigma_Vg_gps = params_.get_double("sigma_Vg_gps");
  double sigma_course_gps = params_.get_double("sigma_course_gps");
  double sigma_accel = params_.get_double("sigma_accel");
  double sigma_pseudo_wind_n = params_.get_double("sigma_pseudo_wind_n");
  double sigma_pseudo_wind_e = params_.get_double("sigma_pseudo_wind_e");
  double sigma_heading = params_.get_double("sigma_heading");
  double frequency = params_.get_double("estimator_update_frequency");
  double Ts = 1.0 / frequency;
  float lpf_a = params_.get_double("lpf_a");
  float lpf_a1 = params_.get_double("lpf_a1");

  R_accel_ = Eigen::Matrix3f::Identity() * pow(sigma_accel, 2);

  R_p_(0, 0) = powf(sigma_n_gps, 2);
  R_p_(1, 1) = powf(sigma_e_gps, 2);
  R_p_(2, 2) = powf(sigma_Vg_gps, 2);
  R_p_(3, 3) = powf(sigma_course_gps, 2);
  R_p_(4, 4) = sigma_pseudo_wind_n;
  R_p_(5, 5) = sigma_pseudo_wind_e;
  R_p_(6, 6) = sigma_heading;

  alpha_ = exp(-lpf_a * Ts);
  alpha1_ = exp(-lpf_a1 * Ts);
}

void EstimatorContinuousDiscrete::estimate(const Input & input, Output & output)
{
  // For readability, declare the parameters here
  double rho = params_.get_double("rho");
  double gravity = params_.get_double("gravity");
  double frequency = params_.get_double("estimator_update_frequency");
  double gps_n_lim = params_.get_double("gps_n_lim");
  double gps_e_lim = params_.get_double("gps_e_lim");
  double Ts = 1.0 / frequency;

  // Inits R matrix and alpha values with ROS2 parameters
  update_measurement_model_parameters();

  // low pass filter gyros to estimate angular rates
  lpf_gyro_x_ = alpha_ * lpf_gyro_x_ + (1 - alpha_) * input.gyro_x;
  lpf_gyro_y_ = alpha_ * lpf_gyro_y_ + (1 - alpha_) * input.gyro_y;
  lpf_gyro_z_ = alpha_ * lpf_gyro_z_ + (1 - alpha_) * input.gyro_z;

  float phat = lpf_gyro_x_;
  float qhat = lpf_gyro_y_;
  float rhat = lpf_gyro_z_;

  // These states will allow us to propagate our state model for pitch and roll.
  Eigen::Vector3f angular_rates;
  angular_rates << phat, qhat, rhat;

  // low pass filter static pressure sensor and invert to esimate altitude
  lpf_static_ = alpha1_ * lpf_static_ + (1 - alpha1_) * input.static_pres;
  float hhat = lpf_static_ / rho / gravity;

  if (input.static_pres == 0.0
      || baro_init_ == false) { // Catch the edge case for if pressure measured is zero.
    hhat = 0.0;
  }

  // low pass filter diff pressure sensor and invert to estimate va
  lpf_diff_ = alpha1_ * lpf_diff_ + (1 - alpha1_) * input.diff_pres;

  // when the plane isn't moving or moving slowly, the noise in the sensor
  // will cause the differential pressure to go negative. This will catch
  // those cases.
  if (lpf_diff_ <= 0)
    lpf_diff_ = 0.000001;

  float vahat = sqrt(2 / rho * lpf_diff_);

  // low pass filter accelerometers
  lpf_accel_x_ = alpha_ * lpf_accel_x_ + (1 - alpha_) * input.accel_x;
  lpf_accel_y_ = alpha_ * lpf_accel_y_ + (1 - alpha_) * input.accel_y;
  lpf_accel_z_ = alpha_ * lpf_accel_z_ + (1 - alpha_) * input.accel_z;

  // These are the current states that will allow us to predict our measurement.
  Eigen::Vector4f att_curr_state_info;
  att_curr_state_info << angular_rates, vahat;

  // The sensor measurements that we can compare to our preditions.
  Eigen::Vector3f y_att;
  y_att << lpf_accel_x_, lpf_accel_y_, lpf_accel_z_;

  // ATTITUDE (ROLL AND PITCH) ESTIMATION
  // Prediction step
  std::tie(P_a_, xhat_a_) =
    propagate_model(xhat_a_, attitude_dynamics_model, attitude_jacobian_model, angular_rates,
                    attitude_input_jacobian_model, P_a_, Q_a_, Q_g_, Ts);

  // Measurement update
  std::tie(P_a_, xhat_a_) = measurement_update(
    xhat_a_, att_curr_state_info, attitude_measurement_model, y_att,
    attitude_measurement_jacobian_model, // TODO: change these std::ties to just pass by reference.
    R_accel_, P_a_);

  // Check the estimate for errors
  check_xhat_a();

  // Store esimate for later use.
  phihat_ = xhat_a_(0);
  thetahat_ = xhat_a_(1);

  // Implement continous-discrete EKF to estimate pn, pe, chi, Vg, wn, we
  // Prediction step

  if (fabsf(xhat_p_(2)) < 0.01f) {
    xhat_p_(2) = 0.01; // prevent divide by zero
  }

  // These are the state that will allow us to propagate our state model for the position state.
  Eigen::Vector<float, 6> attitude_states;
  attitude_states << angular_rates, xhat_a_(0), xhat_a_(1), vahat;

  // POSITION AND COURSE ESTIMATION
  // Prediction step
  std::tie(P_p_, xhat_p_) =
    propagate_model(xhat_p_, position_dynamics_model, position_jacobian_model, attitude_states,
                    position_input_jacobian_model, P_p_, Q_p_, Eigen::MatrixXf::Zero(7, 7), Ts);

  // Check wrapping of the heading and course.
  xhat_p_(3) = wrap_within_180(0.0, xhat_p_(3));
  xhat_p_(6) = wrap_within_180(0.0, xhat_p_(6));
  if (xhat_p_(3) > radians(180.0f) || xhat_p_(3) < radians(-180.0f)) {
    RCLCPP_WARN(this->get_logger(), "Course estimate not wrapped from -pi to pi");
    xhat_p_(3) = 0;
  }
  if (xhat_p_(6) > radians(180.0f) || xhat_p_(6) < radians(-180.0f)) {
    RCLCPP_WARN(this->get_logger(), "Psi estimate not wrapped from -pi to pi");
    xhat_p_(6) = 0;
  }

  // Measurement updates.
  // Only update if new GPS information is available.
  if (input.gps_new) {
    Eigen::VectorXf pos_curr_state_info(1);
    pos_curr_state_info << vahat;

    //wrap course measurement
    float gps_course = fmodf(input.gps_course, radians(360.0f));
    gps_course = wrap_within_180(xhat_p_(3), gps_course);

    // Measurements for the postional states.
    Eigen::Vector<float, 6> y_pos;
    y_pos << input.gps_n, input.gps_e, input.gps_Vg, gps_course, 0.0, 0.0;

    // Update the state and covariance with based on the predicted and actual measurements.
    std::tie(P_p_, xhat_p_) =
      measurement_update(xhat_p_, pos_curr_state_info, position_measurement_model, y_pos,
                         position_measurement_jacobian_model, R_p_, P_p_);

    if (xhat_p_(0) > gps_n_lim || xhat_p_(0) < -gps_n_lim) {
      RCLCPP_WARN(this->get_logger(), "gps n limit reached");
      xhat_p_(0) = input.gps_n;
    }
    if (xhat_p_(1) > gps_e_lim || xhat_p_(1) < -gps_e_lim) {
      RCLCPP_WARN(this->get_logger(), "gps e limit reached");
      xhat_p_(1) = input.gps_e;
    }
  }

  bool problem = false;
  int prob_index;
  for (int i = 0; i < 7; i++) {
    if (!std::isfinite(xhat_p_(i))) {
      if (!problem) {
        problem = true;
        prob_index = i;
      }
      switch (i) {
        case 0:
          xhat_p_(i) = input.gps_n;
          break;
        case 1:
          xhat_p_(i) = input.gps_e;
          break;
        case 2:
          xhat_p_(i) = input.gps_Vg;
          break;
        case 3:
          xhat_p_(i) = input.gps_course;
          break;
        case 6:
          xhat_p_(i) = input.gps_course;
          break;
        default:
          xhat_p_(i) = 0;
      }

      initialize_state_covariances();
    }
  }
  if (problem) {
    RCLCPP_WARN(this->get_logger(), "position estimator reinitialized due to non-finite state %d",
                prob_index);
  }
  if (xhat_p_(6) - xhat_p_(3) > radians(360.0f) || xhat_p_(6) - xhat_p_(3) < radians(-360.0f)) {
    xhat_p_(6) = fmodf(xhat_p_(6), 2.0 * M_PI);
  }

  float pnhat = xhat_p_(0);
  float pehat = xhat_p_(1);
  float Vghat = xhat_p_(2);
  float chihat = xhat_p_(3);
  float wnhat = xhat_p_(4);
  float wehat = xhat_p_(5);
  float psihat = xhat_p_(6);

  output.pn = pnhat;
  output.pe = pehat;
  output.h = hhat;
  output.va = vahat;
  output.alpha = 0;
  output.beta = 0;
  output.phi = phihat_;
  output.theta = thetahat_;
  output.chi = chihat;
  output.p = phat;
  output.q = qhat;
  output.r = rhat;
  output.Vg = Vghat;
  output.wn = wnhat;
  output.we = wehat;
  output.psi = psihat;
}

Eigen::VectorXf
EstimatorContinuousDiscrete::attitude_dynamics(const Eigen::VectorXf & state,
                                               const Eigen::VectorXf & angular_rates)
{
  float cp = cosf(state(0)); // cos(phi)
  float sp = sinf(state(0)); // sin(phi)
  float tt = tanf(state(1)); // tan(theta)

  float p = angular_rates(0);
  float q = angular_rates(1);
  float r = angular_rates(2);

  Eigen::Vector2f f;

  f(0) = p + (q * sp + r * cp) * tt;
  f(1) = q * cp - r * sp;

  return f;
}

Eigen::VectorXf EstimatorContinuousDiscrete::position_dynamics(const Eigen::VectorXf & state,
                                                               const Eigen::VectorXf & measurements)
{

  double gravity = params_.get_double("gravity");

  float Vg = state(2);
  float chi = state(3);
  float wn = state(4);
  float we = state(5);
  float psi = state(6);

  float p = measurements(0);
  float q = measurements(1);
  float r = measurements(2);
  float phi = measurements(3);
  float theta = measurements(4);
  float va = measurements(5);

  float psidot = (q * sinf(phi) + r * cosf(phi)) / cosf(theta);

  float Vgdot = va / Vg * psidot * (we * cosf(psi) - wn * sinf(psi));

  Eigen::VectorXf f;
  f = Eigen::VectorXf::Zero(7);

  f(0) = state(2) * cosf(state(3));
  f(1) = state(2) * sinf(state(3));
  f(2) = Vgdot;
  f(3) = gravity / state(2) * tanf(phi) * cosf(chi - psi);
  f(6) = psidot;

  return f;
}

Eigen::MatrixXf
EstimatorContinuousDiscrete::attitude_jacobian(const Eigen::VectorXf & state,
                                               const Eigen::VectorXf & angular_rates)
{
  float cp = cosf(state(0)); // cos(phi)
  float sp = sinf(state(0)); // sin(phi)
  float tt = tanf(state(1)); // tan(theta)
  float ct = cosf(state(1)); // cos(theta)

  float q = angular_rates(1);
  float r = angular_rates(2);

  Eigen::Matrix2f A = Eigen::Matrix2f::Zero();
  A(0, 0) = (q * cp - r * sp) * tt;
  A(0, 1) = (q * sp + r * cp) / ct / ct;
  A(1, 0) = -q * sp - r * cp;

  return A;
}

Eigen::MatrixXf EstimatorContinuousDiscrete::position_jacobian(const Eigen::VectorXf & state,
                                                               const Eigen::VectorXf & measurements)
{
  double gravity = params_.get_double("gravity");

  float p = measurements(0);
  float q = measurements(1);
  float r = measurements(2);
  float phi = measurements(3);
  float theta = measurements(4);
  float va = measurements(5);

  float Vg = state(2);
  float chi = state(3);
  float wn = state(4);
  float we = state(5);
  float psi = state(6);

  float psidot = (q * sinf(phi) + r * cosf(phi)) / cosf(theta);

  float tmp = -psidot * va * (state(4) * cosf(state(6)) + state(5) * sinf(state(6))) / state(2);

  float Vgdot = va / Vg * psidot * (wn * cosf(psi) - we * sinf(psi));

  Eigen::MatrixXf A;
  A = Eigen::MatrixXf::Zero(7, 7);
  A(0, 2) = cos(state(3));
  A(0, 3) = -state(2) * sinf(state(3));
  A(1, 2) = sin(state(3));
  A(1, 3) = state(2) * cosf(state(3));
  A(2, 2) = -Vgdot / state(2);
  A(2, 4) = -psidot * va * sinf(state(6)) / state(2);
  A(2, 5) = psidot * va * cosf(state(6)) / state(2);
  A(2, 6) = tmp;
  A(3, 2) = -gravity / powf(state(2), 2) * tanf(phihat_);

  return A;
}

Eigen::MatrixXf EstimatorContinuousDiscrete::attitude_input_jacobian(const Eigen::VectorXf & state,
                                                                     const Eigen::VectorXf & inputs)
{
  float cp = cosf(state(0)); // cos(phi)
  float sp = sinf(state(0)); // sin(phi)
  float tt = tanf(state(1)); // tan(theta)

  Eigen::Matrix<float, 2, 3> G;
  G << 1, sp * tt, cp * tt, 0.0, cp, -sp;

  return G;
}

Eigen::MatrixXf EstimatorContinuousDiscrete::position_input_jacobian(const Eigen::VectorXf & state,
                                                                     const Eigen::VectorXf & inputs)
{

  Eigen::MatrixXf G;
  G = Eigen::MatrixXf::Zero(7, 7);

  return G;
}

Eigen::VectorXf
EstimatorContinuousDiscrete::attitude_measurement_prediction(const Eigen::VectorXf & state,
                                                             const Eigen::VectorXf & inputs)
{
  double gravity = params_.get_double("gravity");
  float cp = cosf(state(0)); // cos(phi)
  float sp = sinf(state(0)); // sin(phi)
  float st = sinf(state(1)); // sin(theta)
  float ct = cosf(state(1)); // cos(theta)

  float p = inputs(0);
  float q = inputs(1);
  float r = inputs(2);
  float va = inputs(3);

  Eigen::Vector3f h;
  h = Eigen::Vector3f::Zero(3);
  h(0) = q * va * st + gravity * st;
  h(1) = r * va * ct - p * va * st - gravity * ct * sp;
  h(2) = -q * va * ct - gravity * ct * cp;

  return h;
}

Eigen::VectorXf
EstimatorContinuousDiscrete::position_measurement_prediction(const Eigen::VectorXf & state,
                                                             const Eigen::VectorXf & input)
{
  float va = input(0);

  Eigen::VectorXf h = Eigen::VectorXf::Zero(6);

  // GPS north
  h(0) = state(0);

  // GPS east
  h(1) = state(1);

  // GPS ground speed
  h(2) = state(2);

  // GPS course
  h(3) = state(3);

  // Pseudo Measurement north
  h(4) = va * cosf(state(6)) + state(4) - state(2) * cosf(state(3));

  // Pseudo Measurement east
  h(5) = va * sinf(state(6)) + state(5) - state(2) * sinf(state(3));

  // To add a new measurement, simply use the state and any input you need as another entry to h. Be sure to update the measurement jacobian C.

  return h;
}

Eigen::MatrixXf
EstimatorContinuousDiscrete::position_measurement_jacobian(const Eigen::VectorXf & state,
                                                           const Eigen::VectorXf & input)
{
  float va = input(0);

  Eigen::MatrixXf C = Eigen::MatrixXf::Zero(6, 7);

  // GPS north
  C(0, 0) = 1;

  // GPS east
  C(1, 1) = 1;

  // GPS ground speed
  C(2, 2) = 1;

  // GPS course
  C(3, 3) = 1;

  // Pseudo Measurement north
  C(4, 2) = -cos(state(3));
  C(4, 3) = state(2) * sinf(state(3));
  C(4, 4) = 1;
  C(4, 6) = -va * sinf(state(6));

  // Pseudo Measurement east
  C(5, 2) = -sin(state(3));
  C(5, 3) = -state(2) * cosf(state(3));
  C(5, 5) = 1;
  C(5, 6) = va * cosf(state(6));

  // To add a new measurement use the inputs and the state to add another row to the matrix C. Be sure to update the measurment prediction vector h.

  return C;
}

Eigen::MatrixXf
EstimatorContinuousDiscrete::attitude_measurement_jacobian(const Eigen::VectorXf & state,
                                                           const Eigen::VectorXf & inputs)
{
  double gravity = params_.get_double("gravity");
  float cp = cosf(state(0));
  float sp = sinf(state(0));
  float ct = cosf(state(1));
  float st = sinf(state(1));

  float p = inputs(0);
  float q = inputs(1);
  float r = inputs(2);
  float va = inputs(3);

  Eigen::Matrix<float, 3, 2> C;

  C << 0.0, q * va * ct + gravity * ct, -gravity * cp * ct,
    -r * va * st - p * va * ct + gravity * sp * st, gravity * sp * ct, (q * va + gravity * cp) * st;

  return C;
}

void EstimatorContinuousDiscrete::check_xhat_a()
{
  double max_phi = params_.get_double("max_estimated_phi");
  double max_theta = params_.get_double("max_estimated_theta");
  double buff = params_.get_double("estimator_max_buffer");

  if (xhat_a_(0) > radians(85.0) || xhat_a_(0) < radians(-85.0) || !std::isfinite(xhat_a_(0))) {

    if (!std::isfinite(xhat_a_(0))) {
      xhat_a_(0) = 0;
      P_a_ = Eigen::Matrix2f::Identity();
      P_a_ *= powf(radians(20.0f), 2);
      RCLCPP_WARN(this->get_logger(), "attiude estimator reinitialized due to non-finite roll");
    } else if (xhat_a_(0) > radians(max_phi)) {
      xhat_a_(0) = radians(max_phi - buff);
      RCLCPP_WARN(this->get_logger(), "max roll angle");
    } else if (xhat_a_(0) < radians(-max_phi)) {
      xhat_a_(0) = radians(-max_phi + buff);
      RCLCPP_WARN(this->get_logger(), "min roll angle");
    }
  }

  if (!std::isfinite(xhat_a_(1))) {
    xhat_a_(1) = 0;
    P_a_ = Eigen::Matrix2f::Identity();
    P_a_ *= powf(radians(20.0f), 2);
    RCLCPP_WARN(this->get_logger(), "attiude estimator reinitialized due to non-finite pitch");
  } else if (xhat_a_(1) > radians(max_theta)) {
    xhat_a_(1) = radians(max_theta - buff);
    RCLCPP_WARN(this->get_logger(), "max pitch angle");
  } else if (xhat_a_(1) < radians(-max_theta)) {
    xhat_a_(1) = radians(-max_theta + buff);
    RCLCPP_WARN(this->get_logger(), "min pitch angle");
  }
}

void EstimatorContinuousDiscrete::declare_parameters()
{
  params_.declare_double("sigma_n_gps", .01);
  params_.declare_double("sigma_e_gps", .01);
  params_.declare_double("sigma_Vg_gps", .005);
  params_.declare_double("sigma_course_gps", .005 / 20);
  params_.declare_double("sigma_accel", .0025 * 9.81);
  params_.declare_double("sigma_pseudo_wind_n", 0.01);
  params_.declare_double("sigma_pseudo_wind_e", 0.01);
  params_.declare_double("sigma_heading", 0.01);
  params_.declare_double("lpf_a", 50.0);
  params_.declare_double("lpf_a1", 8.0);
  params_.declare_double("gps_n_lim", 10000.);
  params_.declare_double("gps_e_lim", 10000.);

  params_.declare_double("roll_process_noise", 0.0001);     // Radians?, should be already squared
  params_.declare_double("pitch_process_noise", 0.0000001); // Radians?, already squared
  params_.declare_double("gyro_process_noise", 0.13);       // Deg, not squared
  params_.declare_double("pos_process_noise", 0.1);         // already squared

  params_.declare_double("attitude_initial_cov", 5.0); // Deg, not squared
  params_.declare_double("pos_n_initial_cov", 0.03);
  params_.declare_double("pos_e_initial_cov", 0.03);
  params_.declare_double("vg_initial_cov", 0.01);
  params_.declare_double("chi_initial_cov", 5.0); // Deg
  params_.declare_double("wind_n_initial_cov", 0.04);
  params_.declare_double("wind_e_initial_cov", 0.04);
  params_.declare_double("psi_initial_cov", 5.0); // Deg

  params_.declare_int("num_propagation_steps", 10);

  params_.declare_double("max_estimated_phi", 85.0);   // Deg
  params_.declare_double("max_estimated_theta", 80.0); // Deg
  params_.declare_double("estimator_max_buffer", 3.0); // Deg
}

void EstimatorContinuousDiscrete::bind_functions()
{
  // This creates references to the functions that are necessary estimate. This means we can pass them to the EKF class's functions.
  // std::bind creates a forwarding reference to a function. So when we pass the binding object to another method, that method can call the
  // original function.
  attitude_dynamics_model = std::bind(&EstimatorContinuousDiscrete::attitude_dynamics, this,
                                      std::placeholders::_1, std::placeholders::_2);
  attitude_jacobian_model = std::bind(&EstimatorContinuousDiscrete::attitude_jacobian, this,
                                      std::placeholders::_1, std::placeholders::_2);
  attitude_input_jacobian_model = std::bind(&EstimatorContinuousDiscrete::attitude_input_jacobian,
                                            this, std::placeholders::_1, std::placeholders::_2);
  attitude_measurement_model =
    std::bind(&EstimatorContinuousDiscrete::attitude_measurement_prediction, this,
              std::placeholders::_1, std::placeholders::_2);
  attitude_measurement_jacobian_model =
    std::bind(&EstimatorContinuousDiscrete::attitude_measurement_jacobian, this,
              std::placeholders::_1, std::placeholders::_2);
  position_dynamics_model = std::bind(&EstimatorContinuousDiscrete::position_dynamics, this,
                                      std::placeholders::_1, std::placeholders::_2);
  position_jacobian_model = std::bind(&EstimatorContinuousDiscrete::position_jacobian, this,
                                      std::placeholders::_1, std::placeholders::_2);
  position_input_jacobian_model = std::bind(&EstimatorContinuousDiscrete::position_input_jacobian,
                                            this, std::placeholders::_1, std::placeholders::_2);
  position_measurement_model =
    std::bind(&EstimatorContinuousDiscrete::position_measurement_prediction, this,
              std::placeholders::_1, std::placeholders::_2);
  position_measurement_jacobian_model =
    std::bind(&EstimatorContinuousDiscrete::position_measurement_jacobian, this,
              std::placeholders::_1, std::placeholders::_2);
}

} // namespace rosplane
