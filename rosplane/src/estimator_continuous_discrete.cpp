#include "estimator_continuous_discrete.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "estimator_ros.hpp"
#include <functional>
#include <tuple>

namespace rosplane
{

float radians(float degrees) { return M_PI * degrees / 180.0; }

double wrap_within_180(double fixed_heading, double wrapped_heading)
{
  // wrapped_heading - number_of_times_to_wrap * 2pi
  return wrapped_heading - floor((wrapped_heading - fixed_heading) / (2 * M_PI) + 0.5) * 2 * M_PI;
}

Eigen::VectorXf estimator_continuous_discrete::attitude_dynamics(const Eigen::VectorXf& state, const Eigen::VectorXf& anglular_rates)
{
    float cp = cosf(state(0)); // cos(phi)
    float sp = sinf(state(0)); // sin(phi)
    float tt = tanf(state(1)); // tan(theta)
    
    float p = anglular_rates(0);
    float q = anglular_rates(1);
    float r = anglular_rates(2);
    
    Eigen::Vector2f f;

    f(0) = p + (q * sp + r * cp) * tt;
    f(1) = q * cp - r * sp;

    return f;
}

Eigen::MatrixXf estimator_continuous_discrete::attitude_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& anglular_rates)
{
    float cp = cosf(state(0)); // cos(phi)
    float sp = sinf(state(0)); // sin(phi)
    float tt = tanf(state(1)); // tan(theta)
    float ct = cosf(state(1)); // cos(theta)
    
    float q = anglular_rates(1);
    float r = anglular_rates(2);

  Eigen::Matrix2f A = Eigen::Matrix2f::Zero();
    A(0, 0) = (q * cp - r * sp) * tt;
    A(0, 1) = (q * sp + r * cp) / ct / ct;
    A(1, 0) = -q * sp - r * cp;

    return A;
}

Eigen::MatrixXf estimator_continuous_discrete::attitude_input_jacobian(const Eigen::VectorXf& state, const Eigen::VectorXf& anglular_rates)
{
  float cp = cosf(state(0)); // cos(phi)
  float sp = sinf(state(0)); // sin(phi)
  float tt = tanf(state(1)); // tan(theta)

  Eigen::Matrix<float, 2, 3> G;
  G << 1, sp * tt, cp * tt, 0.0, cp, -sp;

  return G;
}


estimator_continuous_discrete::estimator_continuous_discrete()
    : estimator_ekf()
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
  params.set_parameters();

  // Initialize covariance matrices from ROS2 parameters
  initialize_uncertainties();

  // Inits R matrix and alpha values with ROS2 parameters
  update_measurement_model_parameters();

  N_ = params.get_int("num_propagation_steps");}

estimator_continuous_discrete::estimator_continuous_discrete(bool use_params) : estimator_continuous_discrete()
{
  double init_lat = params.get_double("init_lat");
  double init_long = params.get_double("init_lon");
  double init_alt = params.get_double("init_alt");
  double init_static = params.get_double("baro_calibration_val");

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

void estimator_continuous_discrete::initialize_state_covariances() {
  double pos_n_initial_cov = params.get_double("pos_n_initial_cov");
  double pos_e_initial_cov = params.get_double("pos_e_initial_cov");
  double vg_initial_cov = params.get_double("vg_initial_cov");
  double chi_initial_cov = params.get_double("chi_initial_cov");
  double wind_n_initial_cov = params.get_double("wind_n_initial_cov");
  double wind_e_initial_cov = params.get_double("wind_e_initial_cov");
  double psi_initial_cov = params.get_double("psi_initial_cov");

  P_p_ = Eigen::MatrixXf::Identity(7, 7);
  P_p_(0, 0) = pos_n_initial_cov;
  P_p_(1, 1) = pos_e_initial_cov;
  P_p_(2, 2) = vg_initial_cov;
  P_p_(3, 3) = radians(chi_initial_cov);
  P_p_(4, 4) = wind_n_initial_cov;
  P_p_(5, 5) = wind_e_initial_cov;
  P_p_(6, 6) = radians(psi_initial_cov);
}

void estimator_continuous_discrete::initialize_uncertainties() {
  double roll_process_noise = params.get_double("roll_process_noise");
  double pitch_process_noise = params.get_double("pitch_process_noise");
  double gyro_process_noise = params.get_double("gyro_process_noise");
  double position_process_noise = params.get_double("pos_process_noise");

  double attitude_initial_cov = params.get_double("attitude_initial_cov");
  
  P_a_ *= powf(radians(attitude_initial_cov), 2);

  Q_a_(0, 0) = roll_process_noise;
  Q_a_(1, 1) = pitch_process_noise;

  Q_g_ *= pow(radians(gyro_process_noise), 2);

  Q_p_ *= position_process_noise;

  initialize_state_covariances();
}

void estimator_continuous_discrete::update_measurement_model_parameters()
{
  // For readability, declare the parameters used in the function here
  double sigma_n_gps = params.get_double("sigma_n_gps");
  double sigma_e_gps = params.get_double("sigma_e_gps");
  double sigma_Vg_gps = params.get_double("sigma_Vg_gps");
  double sigma_course_gps = params.get_double("sigma_course_gps");
  double sigma_accel = params.get_double("sigma_accel");
  double sigma_pseudo_wind_n = params.get_double("sigma_pseudo_wind_n");
  double sigma_pseudo_wind_e = params.get_double("sigma_pseudo_wind_e");
  double sigma_heading = params.get_double("sigma_heading");
  double frequency = params.get_double("estimator_update_frequency");
  double Ts = 1.0 / frequency;
  float lpf_a = params.get_double("lpf_a");
  float lpf_a1 = params.get_double("lpf_a1");

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

void estimator_continuous_discrete::estimate(const input_s & input, output_s & output)
{
  // For readability, declare the parameters here
  double rho = params.get_double("rho");
  double gravity = params.get_double("gravity");
  double frequency = params.get_double("estimator_update_frequency");
  double gps_n_lim = params.get_double("gps_n_lim");
  double gps_e_lim = params.get_double("gps_e_lim");
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
  if (lpf_diff_ <= 0) lpf_diff_ = 0.000001;

  float vahat = sqrt(2 / rho * lpf_diff_);

  // low pass filter accelerometers
  lpf_accel_x_ = alpha_ * lpf_accel_x_ + (1 - alpha_) * input.accel_x;
  lpf_accel_y_ = alpha_ * lpf_accel_y_ + (1 - alpha_) * input.accel_y;
  lpf_accel_z_ = alpha_ * lpf_accel_z_ + (1 - alpha_) * input.accel_z;

  // implement continuous-discrete EKF to estimate roll and pitch angles
  // prediction step
  float cp; // cos(phi)
  float sp; // sin(phi)
  float tt; // tan(thata)
  float ct; // cos(thata)F
  float st; // sin(theta) std::bind(&MyClass::memberFunction, &myObject, std::placeholders::_1)
  
  std::tie(P_a_, xhat_a_) = propagate_model(xhat_a_,
                                            std::bind(&estimator_continuous_discrete::attitude_dynamics, this, std::placeholders::_1, std::placeholders::_2),
                                            std::bind(&estimator_continuous_discrete::attitude_jacobian, this, std::placeholders::_1, std::placeholders::_2),
                                            angular_rates,
                                            std::bind(&estimator_continuous_discrete::attitude_input_jacobian, this, std::placeholders::_1, std::placeholders::_2),
                                            P_a_,
                                            Q_a_,
                                            Q_g_,
                                            Ts);

  // measurement updates
  cp = cosf(xhat_a_(0));
  sp = sinf(xhat_a_(0));
  ct = cosf(xhat_a_(1));
  st = sinf(xhat_a_(1));
  Eigen::Matrix2f I;
  I = Eigen::Matrix2f::Identity();

  h_a_ = Eigen::Vector3f::Zero(3);
  h_a_(0) = qhat * vahat * st + gravity * st;
  h_a_(1) = rhat * vahat * ct - phat * vahat * st - gravity * ct * sp;
  h_a_(2) = -qhat * vahat * ct - gravity * ct * cp;

  C_a_ << 0.0, qhat * vahat * ct + gravity * ct, -gravity * cp * ct,
    -rhat * vahat * st - phat * vahat * ct + gravity * sp * st, gravity * sp * ct,
    (qhat * vahat + gravity * cp) * st;

  // This calculates the Kalman Gain for all of the attitude states at once rather than one at a time.

  Eigen::Vector3f y;

  y << lpf_accel_x_, lpf_accel_y_, lpf_accel_z_;

  Eigen::MatrixXf S_inv = (R_accel_ + C_a_ * P_a_ * C_a_.transpose()).inverse();
  Eigen::MatrixXf L_a_ = P_a_ * C_a_.transpose() * S_inv;
  Eigen::MatrixXf temp = Eigen::MatrixXf::Identity(2, 2) - L_a_ * C_a_;

  P_a_ = temp * P_a_ * temp.transpose() + L_a_ * R_accel_ * L_a_.transpose();
  xhat_a_ = xhat_a_ + L_a_ * (y - h_a_);

  check_xhat_a();

  phihat_ = xhat_a_(0);
  thetahat_ = xhat_a_(1);

  // implement continous-discrete EKF to estimate pn, pe, chi, Vg
  // prediction step
  float psidot, tmp, Vgdot;
  if (fabsf(xhat_p_(2)) < 0.01f) {
    xhat_p_(2) = 0.01; // prevent divide by zero
  }

  for (int i = 0; i < N_; i++) {

    float Vg = xhat_p_(2);
    float chi = xhat_p_(3);
    float wn = xhat_p_(4);
    float we = xhat_p_(5);
    float psi = xhat_p_(6);

    psidot = (qhat * sinf(phihat_) + rhat * cosf(phihat_)) / cosf(thetahat_);

    tmp = -psidot * vahat * (xhat_p_(4) * cosf(xhat_p_(6)) + xhat_p_(5) * sinf(xhat_p_(6)))
      / xhat_p_(2);
    Vgdot = vahat / Vg * psidot * (we * cosf(psi) - wn * sinf(psi));

    f_p_ = Eigen::VectorXf::Zero(7);
    f_p_(0) = xhat_p_(2) * cosf(xhat_p_(3));
    f_p_(1) = xhat_p_(2) * sinf(xhat_p_(3));
    f_p_(2) = Vgdot;
    f_p_(3) = gravity / xhat_p_(2) * tanf(phihat_) * cosf(chi - psi);
    f_p_(6) = psidot;

    xhat_p_ += f_p_ * (Ts / N_);

    A_p_ = Eigen::MatrixXf::Zero(7, 7);
    A_p_(0, 2) = cos(xhat_p_(3));
    A_p_(0, 3) = -xhat_p_(2) * sinf(xhat_p_(3));
    A_p_(1, 2) = sin(xhat_p_(3));
    A_p_(1, 3) = xhat_p_(2) * cosf(xhat_p_(3));
    A_p_(2, 2) = -Vgdot / xhat_p_(2);
    A_p_(2, 4) = -psidot * vahat * sinf(xhat_p_(6)) / xhat_p_(2);
    A_p_(2, 5) = psidot * vahat * cosf(xhat_p_(6)) / xhat_p_(2);
    A_p_(2, 6) = tmp;
    A_p_(3, 2) = -gravity / powf(xhat_p_(2), 2) * tanf(phihat_);

    Eigen::MatrixXf A_d_ = Eigen::MatrixXf::Identity(7, 7) + Ts / N_ * A_p_
      + A_p_ * A_p_ * pow(Ts / N_, 2) / 2.0;

    P_p_ = (A_d_ * P_p_ * A_d_.transpose() + Q_p_ * pow(Ts / N_, 2));
  }

  xhat_p_(3) = wrap_within_180(0.0, xhat_p_(3));
  if (xhat_p_(3) > radians(180.0f) || xhat_p_(3) < radians(-180.0f)) {
    RCLCPP_WARN(this->get_logger(), "Course estimate not wrapped from -pi to pi");
    xhat_p_(3) = 0;
  }

  xhat_p_(6) = wrap_within_180(0.0, xhat_p_(6));
  if (xhat_p_(6) > radians(180.0f) || xhat_p_(6) < radians(-180.0f)) {
    RCLCPP_WARN(this->get_logger(), "Psi estimate not wrapped from -pi to pi");
    xhat_p_(6) = 0;
  }

  // measurement updates
  // These calculate the Kalman gain and applies them to each state individually.
  if (input.gps_new) {

    Eigen::MatrixXf I_p(7, 7);
    I_p = Eigen::MatrixXf::Identity(7, 7);

    // gps North position
    h_p_ = xhat_p_(0);
    C_p_ = Eigen::VectorXf::Zero(7);
    C_p_(0) = 1;
    L_p_ = (P_p_ * C_p_) / (R_p_(0, 0) + (C_p_.transpose() * P_p_ * C_p_));
    P_p_ = (I_p - L_p_ * C_p_.transpose()) * P_p_;
    xhat_p_ = xhat_p_ + L_p_ * (input.gps_n - h_p_);

    // gps East position
    h_p_ = xhat_p_(1);
    C_p_ = Eigen::VectorXf::Zero(7);
    C_p_(1) = 1;
    L_p_ = (P_p_ * C_p_) / (R_p_(1, 1) + (C_p_.transpose() * P_p_ * C_p_));
    P_p_ = (I_p - L_p_ * C_p_.transpose()) * P_p_;
    xhat_p_ = xhat_p_ + L_p_ * (input.gps_e - h_p_);

    // gps ground speed
    h_p_ = xhat_p_(2);
    C_p_ = Eigen::VectorXf::Zero(7);
    C_p_(2) = 1;
    L_p_ = (P_p_ * C_p_) / (R_p_(2, 2) + (C_p_.transpose() * P_p_ * C_p_));
    P_p_ = (I_p - L_p_ * C_p_.transpose()) * P_p_;
    xhat_p_ = xhat_p_ + L_p_ * (input.gps_Vg - h_p_);

    // gps course
    //wrap course measurement
    float gps_course = fmodf(input.gps_course, radians(360.0f));

    gps_course = wrap_within_180(xhat_p_(3), gps_course);
    h_p_ = xhat_p_(3);

    C_p_ = Eigen::VectorXf::Zero(7);
    C_p_(3) = 1;
    L_p_ = (P_p_ * C_p_) / (R_p_(3, 3) + (C_p_.transpose() * P_p_ * C_p_));
    P_p_ = (I_p - L_p_ * C_p_.transpose()) * P_p_;
    xhat_p_ = xhat_p_ + L_p_ * (gps_course - h_p_);

    // pseudo measurement #1 y_1 = va*cos(psi)+wn-Vg*cos(chi)
    h_p_ =
      vahat * cosf(xhat_p_(6)) + xhat_p_(4) - xhat_p_(2) * cosf(xhat_p_(3)); // pseudo measurement
    C_p_ = Eigen::VectorXf::Zero(7);
    C_p_(2) = -cos(xhat_p_(3));
    C_p_(3) = xhat_p_(2) * sinf(xhat_p_(3));
    C_p_(4) = 1;
    C_p_(6) = -vahat * sinf(xhat_p_(6));
    L_p_ = (P_p_ * C_p_) / (R_p_(4, 4) + (C_p_.transpose() * P_p_ * C_p_));
    P_p_ = (I_p - L_p_ * C_p_.transpose()) * P_p_;
    xhat_p_ = xhat_p_ + L_p_ * (0 - h_p_);

    // pseudo measurement #2 y_2 = va*sin(psi) + we - Vg*sin(chi)
    h_p_ =
      vahat * sinf(xhat_p_(6)) + xhat_p_(5) - xhat_p_(2) * sinf(xhat_p_(3)); // pseudo measurement
    C_p_ = Eigen::VectorXf::Zero(7);
    C_p_(2) = -sin(xhat_p_(3));
    C_p_(3) = -xhat_p_(2) * cosf(xhat_p_(3));
    C_p_(5) = 1;
    C_p_(6) = vahat * cosf(xhat_p_(6));
    L_p_ = (P_p_ * C_p_) / (R_p_(5, 5) + (C_p_.transpose() * P_p_ * C_p_));
    P_p_ = (I_p - L_p_ * C_p_.transpose()) * P_p_;
    xhat_p_ = xhat_p_ + L_p_ * (0 - h_p_);

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
    //xhat_p(3) = fmodf(xhat_p(3),2.0*M_PI);
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

void estimator_continuous_discrete::check_xhat_a()
{
  double max_phi = params.get_double("max_estimated_phi");
  double max_theta = params.get_double("max_estimated_theta");
  double buff = params.get_double("estimator_max_buffer");

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

void estimator_continuous_discrete::declare_parameters()
{
  params.declare_double("sigma_n_gps", .01);
  params.declare_double("sigma_e_gps", .01);
  params.declare_double("sigma_Vg_gps", .005);
  params.declare_double("sigma_course_gps", .005 / 20);
  params.declare_double("sigma_accel", .0025 * 9.81);
  params.declare_double("sigma_pseudo_wind_n", 0.01);
  params.declare_double("sigma_pseudo_wind_e", 0.01);
  params.declare_double("sigma_heading", 0.01);
  params.declare_double("lpf_a", 50.0);
  params.declare_double("lpf_a1", 8.0);
  params.declare_double("gps_n_lim", 10000.);
  params.declare_double("gps_e_lim", 10000.);

  params.declare_double("roll_process_noise", 0.0001);   // Radians?, should be already squared
  params.declare_double("pitch_process_noise", 0.0000001);   // Radians?, already squared
  params.declare_double("gyro_process_noise", 0.13);   // Deg, not squared
  params.declare_double("pos_process_noise", 0.1);   // already squared

  params.declare_double("attitude_initial_cov", 5.0); // Deg, not squared
  params.declare_double("pos_n_initial_cov", 0.03);
  params.declare_double("pos_e_initial_cov", 0.03);
  params.declare_double("vg_initial_cov", 0.01);
  params.declare_double("chi_initial_cov", 5.0);  // Deg
  params.declare_double("wind_n_initial_cov", 0.04);
  params.declare_double("wind_e_initial_cov", 0.04);
  params.declare_double("psi_initial_cov", 5.0);  // Deg

  params.declare_int("num_propagation_steps", 10);

  params.declare_double("max_estimated_phi", 85.0); // Deg
  params.declare_double("max_estimated_theta", 80.0); // Deg
  params.declare_double("estimator_max_buffer", 3.0);   // Deg
}

} // namespace rosplane
