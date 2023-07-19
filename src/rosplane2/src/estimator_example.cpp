#include "estimator_base.h"
#include "estimator_example.h"

namespace rosplane2
{

float radians(float degrees)
{
  return M_PI*degrees/180.0;
}

estimator_example::estimator_example() :
  estimator_base(),
  xhat_a_(Eigen::Vector2f::Zero()),
  P_a_(Eigen::Matrix2f::Zero()),
  Q_a_(Eigen::Matrix2f::Identity()),
  Q_g_(Eigen::Matrix3f::Identity()),
  xhat_p_(Eigen::VectorXf::Zero(7)),
  P_p_(Eigen::MatrixXf::Identity(7, 7)),
  Q_p_(Eigen::MatrixXf::Identity(7, 7)),
  R_p_(Eigen::MatrixXf::Zero(7, 7)),
  R_accel_(Eigen::Matrix3f::Identity()),
  f_p_(7),
  A_p_(7, 7),
  C_p_(7),
  L_p_(7)
{
  P_a_ *= powf(radians(5.0f), 2);

  Q_a_(0, 0) = 0.01;
  Q_a_(1, 1) = 0.01;

  Q_g_ *= pow(M_PI*.13/180.0,2); // TODO connect this to the actual params.

  P_p_ = Eigen::MatrixXf::Identity(7, 7);
  P_p_(0, 0) = .03; // TODO Add params for these?
  P_p_(1, 1) = .03;
  P_p_(2, 2) = .01;
  P_p_(3, 3) = radians(5.0f);
  P_p_(4, 4) = .04;
  P_p_(5, 5) = .04;
  P_p_(6, 6) = radians(5.0f);

  Q_p_ *= 0.1f;

  phat_ = 0;
  qhat_ = 0;
  rhat_ = 0;
  phihat_ = 0;
  thetahat_ = 0;
  psihat_ = 0;
  Vwhat_ = 0;

  lpf_static_ = 0.0;
  lpf_diff_ = 0.0;

  N_ = 10;

  alpha_ = 0.0f;
}

void estimator_example::estimate(const params_s &params, const input_s &input, output_s &output)
{
  if (alpha_ == 0.0f) //initailze stuff that comes from params
  {
    R_accel_ = R_accel_ * pow(.0025*9.81, 2); // TODO add ot params

    R_p_(0, 0) = powf(params.sigma_n_gps, 2);
    R_p_(1, 1) = powf(params.sigma_e_gps, 2);
    R_p_(2, 2) = powf(params.sigma_Vg_gps, 2);
    R_p_(3, 3) = powf(params.sigma_course_gps, 2);
    R_p_(4, 4) = 0.01;
    R_p_(5, 5) = 0.01;
    R_p_(6, 6) = 0.01;


    float lpf_a = 50.0;
    float lpf_a1 = 8.0;
    alpha_ = exp(-lpf_a*params.Ts);
    alpha1_ = exp(-lpf_a1*params.Ts);
  }

  // low pass filter gyros to estimate angular rates
  lpf_gyro_x_ = input.gyro_x;
  lpf_gyro_y_ = input.gyro_y; // alpha_*lpf_gyro_y_ + (1 - alpha_)*input.gyro_y;
  lpf_gyro_z_ = alpha_*lpf_gyro_z_ + (1 - alpha_)*input.gyro_z;

  float phat = lpf_gyro_x_;
  float qhat = lpf_gyro_y_;
  float rhat = lpf_gyro_z_;

  // low pass filter static pressure sensor and invert to esimate altitude
  lpf_static_ = alpha1_*lpf_static_ + (1 - alpha1_)*input.static_pres;
  float hhat = lpf_static_/params.rho/params.gravity;

  if(input.static_pres == 0.0 || baro_init_ == false){ // Catch the edge case for if pressure measured is zero.
    hhat = 0.0;
  }

  // low pass filter diff pressure sensor and invert to estimate Va
  lpf_diff_ = input.diff_pres;

  // when the plane isn't moving or moving slowly, the noise in the sensor
  // will cause the differential pressure to go negative. This will catch
  // those cases.
  if (lpf_diff_ <= 0)
    lpf_diff_ = 0.000001;

  float Vahat = sqrt(2/params.rho*lpf_diff_);

  // low pass filter accelerometers
  lpf_accel_x_ = alpha_*lpf_accel_x_ + (1 - alpha_)*input.accel_x;
  lpf_accel_y_ = alpha_*lpf_accel_y_ + (1 - alpha_)*input.accel_y;
  lpf_accel_z_ = alpha_*lpf_accel_z_ + (1 - alpha_)*input.accel_z;

  // implement continuous-discrete EKF to estimate roll and pitch angles
  // prediction step
  float cp; // cos(phi)
  float sp; // sin(phi)
  float tt; // tan(thata)
  float ct; // cos(thata)F
  float st; // sin(theta)
  for (int i = 0; i < N_; i++)
  {

    cp = cosf(xhat_a_(0)); // cos(phi)
    sp = sinf(xhat_a_(0)); // sin(phi)
    tt = tanf(xhat_a_(1)); // tan(theta)
    ct = cosf(xhat_a_(1)); // cos(theta)

    f_a_(0) = phat + (qhat*sp + rhat*cp)*tt;
    f_a_(1) = qhat*cp - rhat*sp;

    xhat_a_ += f_a_*(params.Ts/N_);

    cp = cosf(xhat_a_(0)); // cos(phi)
    sp = sinf(xhat_a_(0)); // sin(phi)
    tt = tanf(xhat_a_(1)); // tan(theta)
    ct = cosf(xhat_a_(1)); // cos(theta)

    A_a_ = Eigen::Matrix2f::Zero();
    A_a_(0, 0) = (qhat*cp - rhat*sp)*tt;
    A_a_(0, 1) = (qhat*sp + rhat*cp)/ct/ct;
    A_a_(1, 0) = -qhat*sp - rhat*cp;

//      RCLCPP_INFO_STREAM(this->get_logger(), "Got here...");


    Eigen::MatrixXf A_d = Eigen::MatrixXf::Identity(2,2) + params.Ts/N_ * A_a_ + pow(params.Ts/N_, 2) / 2.0 * A_a_ * A_a_;

    Eigen::Matrix<float, 2, 3> G;
    G << 1, sp*tt, cp*tt, 0.0, cp, -sp;

    P_a_ = (A_d*P_a_*A_d.transpose() + (Q_a_ + G * Q_g_ * G.transpose())*pow(params.Ts/N_, 2));

//    P_a_ += (A_a_*P_a_ + P_a_*A_a_.transpose() + Q_a_) * (params.Ts/N_);


  }
  // measurement updates
  cp = cosf(xhat_a_(0));
  sp = sinf(xhat_a_(0));
  ct = cosf(xhat_a_(1));
  st = sinf(xhat_a_(1));
  Eigen::Matrix2f I;
  I = Eigen::Matrix2f::Identity();

  h_a_ = Eigen::Vector3f::Zero(3);
  h_a_(0) = qhat * Vahat * st + params.gravity*st;
  h_a_(1) = rhat * Vahat * ct - phat * Vahat * st - params.gravity * ct * sp;
  h_a_(2) = -qhat * Vahat * ct - params.gravity * ct * cp;

  C_a_ << 0.0,                      qhat*Vahat*ct + params.gravity*ct,
          -params.gravity*cp*ct,    -rhat*Vahat*st - phat*Vahat*ct + params.gravity*sp*st,
          params.gravity*sp*ct,     (qhat*Vahat + params.gravity*cp) * st;

  Eigen::Vector3f y;

  y << lpf_accel_x_, lpf_accel_y_, lpf_accel_z_;

  Eigen::MatrixXf S_inv = (R_accel_ + C_a_ * P_a_ * C_a_.transpose()).inverse();

  if ((y-h_a_).transpose() * S_inv * (y-h_a_) < 10000000.0){
      Eigen::MatrixXf L = P_a_ * C_a_.transpose() * S_inv;
      Eigen::MatrixXf temp = Eigen::MatrixXf::Identity(2,2) - L * C_a_;
      P_a_ = temp * P_a_ * temp.transpose() + L * R_accel_ * L.transpose();
      xhat_a_ = xhat_a_ + L * (y - h_a_);
  }

  check_xhat_a();



  phihat_ = xhat_a_(0);
  thetahat_ = xhat_a_(1);

  // implement continous-discrete EKF to estimate pn, pe, chi, Vg
  // prediction step
  float psidot, tmp, Vgdot;
  if (fabsf(xhat_p_(2)) < 0.01f)
  {
    xhat_p_(2) = 0.01; // prevent divide by zero
  }

  for (int i = 0; i < N_; i++)
  {

    float pn = xhat_p_(0);
    float pe = xhat_p_(1);
    float Vg = xhat_p_(2);
    float chi = xhat_p_(3);
    float wn = xhat_p_(4);
    float we = xhat_p_(5);
    float psi = xhat_p_(6);

    psidot = (qhat*sinf(phihat_) + rhat*cosf(phihat_))/cosf(thetahat_);

    tmp = -psidot*Vahat*(xhat_p_(4)*cosf(xhat_p_(6)) + xhat_p_(5)*sinf(xhat_p_(6)))/xhat_p_(2);
    Vgdot = Vahat/Vg * psidot*(we*cosf(psi) - wn*sinf(psi));

    f_p_ = Eigen::VectorXf::Zero(7);
    f_p_(0) = xhat_p_(2)*cosf(xhat_p_(3));
    f_p_(1) = xhat_p_(2)*sinf(xhat_p_(3));
    f_p_(2) = Vgdot;
    f_p_(3) = params.gravity/xhat_p_(2)*tanf(phihat_)*cosf(chi-psi);
    f_p_(6) = psidot;

    xhat_p_ += f_p_*(params.Ts/N_);

    A_p_ = Eigen::MatrixXf::Zero(7, 7);
    A_p_(0, 2) = cos(xhat_p_(3));
    A_p_(0, 3) = -xhat_p_(2)*sinf(xhat_p_(3));
    A_p_(1, 2) = sin(xhat_p_(3));
    A_p_(1, 3) = xhat_p_(2)*cosf(xhat_p_(3));
    A_p_(2, 2) = -Vgdot/xhat_p_(2);
    A_p_(2, 4) = -psidot*Vahat*sinf(xhat_p_(6))/xhat_p_(2);
    A_p_(2, 5) = psidot*Vahat*cosf(xhat_p_(6))/xhat_p_(2);
    A_p_(2, 6) = tmp;
    A_p_(3, 2) = -params.gravity/powf(xhat_p_(2), 2)*tanf(phihat_);


    Eigen::MatrixXf A_d_ = Eigen::MatrixXf::Identity(7,7) + params.Ts/N_*A_p_ + A_p_ * A_p_ * pow(params.Ts/N_, 2)/2.0;

    P_p_ = (A_d_*P_p_*A_d_.transpose() + Q_p_*pow(params.Ts/N_, 2));
  }

    while(xhat_p_(3) > radians(180.0f)) xhat_p_(3) = xhat_p_(3) - radians(360.0f);
    while(xhat_p_(3) < radians(-180.0f)) xhat_p_(3) = xhat_p_(3) + radians(360.0f);
    if(xhat_p_(3) > radians(180.0f) || xhat_p_(3) < radians(-180.0f))
    {
        RCLCPP_WARN(this->get_logger(), "Course estimate not wrapped from -pi to pi");
        xhat_p_(3) = 0;
    }

    while(xhat_p_(6) > radians(180.0f)) xhat_p_(6) = xhat_p_(6) - radians(360.0f);
    while(xhat_p_(6) < radians(-180.0f)) xhat_p_(6) = xhat_p_(6) + radians(360.0f);
    if(xhat_p_(6) > radians(180.0f) || xhat_p_(6) < radians(-180.0f))
    {
        RCLCPP_WARN(this->get_logger(), "Psi estimate not wrapped from -pi to pi");
        xhat_p_(6) = 0;
    }


  // measurement updates
  if (input.gps_new)
  {

    Eigen::MatrixXf I_p(7, 7);
    I_p = Eigen::MatrixXf::Identity(7, 7);

    // gps North position
    h_p_ = xhat_p_(0);
    C_p_ = Eigen::VectorXf::Zero(7);
    C_p_(0) = 1;
    L_p_ = (P_p_*C_p_)/(R_p_(0, 0) + (C_p_.transpose()*P_p_*C_p_));
    P_p_ = (I_p - L_p_*C_p_.transpose())*P_p_;
    xhat_p_ = xhat_p_ + L_p_*(input.gps_n - h_p_);

    // gps East position
    h_p_ = xhat_p_(1);
    C_p_ = Eigen::VectorXf::Zero(7);
    C_p_(1) = 1;
    L_p_ = (P_p_*C_p_)/(R_p_(1, 1) + (C_p_.transpose()*P_p_*C_p_));
    P_p_ = (I_p - L_p_*C_p_.transpose())*P_p_;
    xhat_p_ = xhat_p_ + L_p_*(input.gps_e - h_p_);

    // gps ground speed
    h_p_ = xhat_p_(2);
    C_p_ = Eigen::VectorXf::Zero(7);
    C_p_(2) = 1;
    L_p_ = (P_p_*C_p_)/(R_p_(2, 2) + (C_p_.transpose()*P_p_*C_p_));
    P_p_ = (I_p - L_p_*C_p_.transpose())*P_p_;
    xhat_p_ = xhat_p_ + L_p_*(input.gps_Vg - h_p_);

    // gps course
    //wrap course measurement
    float gps_course = fmodf(input.gps_course, radians(360.0f));

    while (gps_course - xhat_p_(3) > radians(180.0f)) gps_course = gps_course - radians(360.0f);
    while (gps_course - xhat_p_(3) < radians(-180.0f)) gps_course = gps_course + radians(360.0f);
    h_p_ = xhat_p_(3);

    C_p_ = Eigen::VectorXf::Zero(7);
    C_p_(3) = 1;
    L_p_ = (P_p_*C_p_)/(R_p_(3, 3) + (C_p_.transpose()*P_p_*C_p_));
    P_p_ = (I_p - L_p_*C_p_.transpose())*P_p_;
    xhat_p_ = xhat_p_ + L_p_*(gps_course - h_p_);

    // pseudo measurement #1 y_1 = Va*cos(psi)+wn-Vg*cos(chi)
    h_p_ = Vahat*cosf(xhat_p_(6)) + xhat_p_(4) - xhat_p_(2)*cosf(xhat_p_(3));  // pseudo measurement
    C_p_ = Eigen::VectorXf::Zero(7);
    C_p_(2) = -cos(xhat_p_(3));
    C_p_(3) = xhat_p_(2)*sinf(xhat_p_(3));
    C_p_(4) = 1;
    C_p_(6) = -Vahat*sinf(xhat_p_(6));
    L_p_ = (P_p_*C_p_)/(R_p_(4,4) + (C_p_.transpose()*P_p_*C_p_));
    P_p_ = (I_p - L_p_*C_p_.transpose())*P_p_;
    xhat_p_ = xhat_p_ + L_p_*(0 - h_p_);


    // pseudo measurement #2 y_2 = Va*sin(psi) + we - Vg*sin(chi)
    h_p_ = Vahat*sinf(xhat_p_(6))+xhat_p_(5)-xhat_p_(2)*sinf(xhat_p_(3));  // pseudo measurement
    C_p_ = Eigen::VectorXf::Zero(7);
    C_p_(2) = -sin(xhat_p_(3));
    C_p_(3) = -xhat_p_(2)*cosf(xhat_p_(3));
    C_p_(5) = 1;
    C_p_(6) = Vahat*cosf(xhat_p_(6));
    L_p_ = (P_p_*C_p_)/(R_p_(5,5) + (C_p_.transpose()*P_p_*C_p_));
    P_p_ = (I_p - L_p_*C_p_.transpose())*P_p_;
    xhat_p_ = xhat_p_ + L_p_*(0 - h_p_);

    if (xhat_p_(0) > 10000 || xhat_p_(0) < -10000)
    {
      RCLCPP_WARN(this->get_logger(), "gps n limit reached");
      xhat_p_(0) = input.gps_n;
    }
    if (xhat_p_(1) > 10000 || xhat_p_(1) < -10000)
    {
      RCLCPP_WARN(this->get_logger(), "gps e limit reached");
      xhat_p_(1) = input.gps_e;
    }
  }

  bool problem = false;
  int prob_index;
  for (int i = 0; i < 7; i++)
  {
    if (!std::isfinite(xhat_p_(i)))
    {
      if (!problem)
      {
        problem = true;
        prob_index = i;
      }
      switch (i)
      {
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
      P_p_ = Eigen::MatrixXf::Identity(7, 7);
      P_p_(0, 0) = .03;
      P_p_(1, 1) = .03;
      P_p_(2, 2) = .01;
      P_p_(3, 3) = radians(5.0f);
      P_p_(4, 4) = .04;
      P_p_(5, 5) = .04;
      P_p_(6, 6) = radians(5.0f);
    }
  }
  if (problem)
  {
//    RCLCPP_WARN(this->get_logger(), "position estimator reinitialized due to non-finite state %d", prob_index); TODO come back and track down why pn is going infinite.
  }
  if (xhat_p_(6) - xhat_p_(3) > radians(360.0f) || xhat_p_(6) - xhat_p_(3) < radians(-360.0f))
  {
    //xhat_p(3) = fmodf(xhat_p(3),2.0*M_PI);
    xhat_p_(6) = fmodf(xhat_p_(6), 2.0*M_PI);
  }

  float pnhat = xhat_p_(0);
  float pehat = xhat_p_(1);
  float Vghat = xhat_p_(2);
  float chihat = xhat_p_(3);
  float wnhat = xhat_p_(4);
  float wehat = xhat_p_(5);
  float psihat = xhat_p_(6);


//    RCLCPP_INFO_STREAM(this->get_logger(), "chihat: " << chihat);

  output.pn = pnhat;
  output.pe = pehat;
  output.h = hhat;
  output.Va = Vahat;
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

void estimator_example::check_xhat_a()
{
  if (xhat_a_(0) > radians(85.0) || xhat_a_(0) < radians(-85.0) || !std::isfinite(xhat_a_(0)))
  {
    if (!std::isfinite(xhat_a_(0)))
    {
      xhat_a_(0) = 0;
      P_a_ = Eigen::Matrix2f::Identity();
      P_a_ *= powf(radians(20.0f), 2);
      RCLCPP_WARN(this->get_logger(), "attiude estimator reinitialized due to non-finite roll");
    }
    else if (xhat_a_(0) > radians(85.0))
    {
      xhat_a_(0) = radians(82.0);
      RCLCPP_WARN(this->get_logger(), "max roll angle");
    }
    else if (xhat_a_(0) < radians(-85.0))
    {
      xhat_a_(0) = radians(-82.0);
      RCLCPP_WARN(this->get_logger(), "min roll angle");
    }
  }
  if (xhat_a_(1) > radians(80.0) || xhat_a_(1) < radians(-80.0) || !std::isfinite(xhat_a_(1)))
  {
    if (!std::isfinite(xhat_a_(1)))
    {
      xhat_a_(1) = 0;
      P_a_ = Eigen::Matrix2f::Identity();
      P_a_ *= powf(radians(20.0f), 2);
      RCLCPP_WARN(this->get_logger(), "attiude estimator reinitialized due to non-finite pitch");
    }
    else if (xhat_a_(1) > radians(80.0))
    {
      xhat_a_(1) = radians(77.0);
      RCLCPP_WARN(this->get_logger(), "max pitch angle");
    }
    else if (xhat_a_(1) < radians(-80.0))
    {
      xhat_a_(1) = radians(-77.0);
      RCLCPP_WARN(this->get_logger(), "min pitch angle");
    }
  }
}

} //end namespace
