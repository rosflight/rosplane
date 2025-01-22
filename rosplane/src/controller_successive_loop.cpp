#include <cmath>
#include <iostream>

#include "controller_successive_loop.hpp"

namespace rosplane
{

double wrap_within_180(double fixed_heading, double wrapped_heading)
{
  // wrapped_heading - number_of_times_to_wrap * 2pi
  return wrapped_heading - floor((wrapped_heading - fixed_heading) / (2 * M_PI) + 0.5) * 2 * M_PI;
}

ControllerSucessiveLoop::ControllerSucessiveLoop()
{
  // Initialize course hold, roll hold and pitch hold errors and integrators to zero.
  c_error_ = 0;
  c_integrator_ = 0;
  r_error_ = 0;
  r_integrator = 0;
  p_error_ = 0;
  p_integrator_ = 0;

  // Declare parameters associated with this controller, controller_state_machine
  declare_parameters();
  // Set parameters according to the parameters in the launch file, otherwise use the default values
  params_.set_parameters();
}

void ControllerSucessiveLoop::take_off(const Input & input, Output & output)
{
  // Run lateral and longitudinal controls.
  take_off_lateral_control(input, output);
  take_off_longitudinal_control(input, output);
}

void ControllerSucessiveLoop::take_off_exit()
{
  // Put any code that should run as the airplane exits take off mode.
}

void ControllerSucessiveLoop::climb(const Input & input, Output & output)
{
  // Run lateral and longitudinal controls.
  climb_lateral_control(input, output);
  climb_longitudinal_control(input, output);
}

void ControllerSucessiveLoop::climb_exit()
{
  // Reset differentiators, integrators and errors.
  at_error_ = 0;
  at_integrator_ = 0;
  at_differentiator_ = 0;
  a_error_ = 0;
  a_integrator_ = 0;
  a_differentiator_ = 0;
}

void ControllerSucessiveLoop::altitude_hold(const Input & input, Output & output)
{
  // Run lateral and longitudinal controls.
  alt_hold_lateral_control(input, output);
  alt_hold_longitudinal_control(input, output);
}

void ControllerSucessiveLoop::altitude_hold_exit()
{
  // Reset integrators.
  c_integrator_ = 0;
}

void ControllerSucessiveLoop::alt_hold_lateral_control(const Input & input, Output & output)
{
  // For readability, declare parameters here that will be used in this function
  bool roll_override = params_.get_bool("roll_command_override"); // Declared in controller_base

  // Set rudder command to zero, can use coordinated_turn_hold if implemented.
  // Find commanded roll angle in order to achieve commanded course.
  // Find aileron deflection required to achieve required roll angle.
  output.delta_r = yaw_damper(input.r); //coordinated_turn_hold(input.beta, params)
  output.phi_c = course_hold(input.chi_c, input.chi, input.phi_ff, input.r);

  if (roll_override) {
    output.phi_c = get_phi_c();
  }

  output.delta_a = roll_hold(output.phi_c, input.phi, input.p);
}

void ControllerSucessiveLoop::alt_hold_longitudinal_control(const Input & input, Output & output)
{
  // For readability, declare parameters here that will be used in this function
  double alt_hz = params_.get_double("alt_hz"); // Declared in controller_state_machine
  bool pitch_override = params_.get_bool("pitch_command_override"); // Declared in controller_base

  // Saturate the altitude command.
  double adjusted_hc = adjust_h_c(input.h_c, input.h, alt_hz);

  // Control airspeed with throttle loop and altitude with commanded pitch and drive aircraft to commanded pitch.
  output.delta_t = airspeed_with_throttle_hold(input.va_c, input.va);
  output.theta_c = altitude_hold_control(adjusted_hc, input.h);

  if (pitch_override) {
    output.theta_c = get_theta_c();
  }

  output.delta_e = pitch_hold(output.theta_c, input.theta, input.q);
}

void ControllerSucessiveLoop::climb_lateral_control(const Input & input, Output & output)
{
  // Maintain straight flight while gaining altitude.
  output.phi_c = 0;
  output.delta_a = roll_hold(output.phi_c, input.phi, input.p);
  output.delta_r = yaw_damper(input.r);
}

void ControllerSucessiveLoop::climb_longitudinal_control(const Input & input, Output & output)
{
  // For readability, declare parameters here that will be used in this function
  double alt_hz = params_.get_double("alt_hz"); // Declared in controller_state_machine

  // Saturate the altitude command.
  double adjusted_hc = adjust_h_c(input.h_c, input.h, alt_hz);

  // Find the control efforts for throttle and find the commanded pitch angle.
  output.delta_t = airspeed_with_throttle_hold(input.va_c, input.va);
  output.theta_c = altitude_hold_control(adjusted_hc, input.h);
  output.delta_e = pitch_hold(output.theta_c, input.theta, input.q);
}

void ControllerSucessiveLoop::take_off_lateral_control(const Input & input, Output & output)
{
  // In the take-off zone maintain level straight flight by commanding a roll angle of 0 and rudder of 0.
  output.delta_r = 0.0;
  output.phi_c = 0;
  output.delta_a = roll_hold(output.phi_c, input.phi, input.p);
}

void ControllerSucessiveLoop::take_off_longitudinal_control(const Input & input, Output & output)
{
  // For readability, declare parameters here that will be used in this function
  double max_takeoff_throttle = params_.get_double("max_takeoff_throttle");
  double cmd_takeoff_pitch = params_.get_double("cmd_takeoff_pitch");

  // Set throttle to not overshoot altitude.
  output.delta_t = sat(airspeed_with_throttle_hold(input.va_c, input.va), max_takeoff_throttle, 0);

  // Command a shallow pitch angle to gain altitude.
  output.theta_c = cmd_takeoff_pitch * M_PI / 180.0;
  output.delta_e = pitch_hold(output.theta_c, input.theta, input.q);
}

/// All the following control loops follow this basic outline.
/*
    float controller_successive_loop::pid_control(float command_val, float actual_val, float rate, // Not all loops use rate.
                                          const params_s &params)
    {
      // Find the error between the commanded and actual value.
      float error = commanded_val - actual_val;

      float Ts = 1.0/params.frequency;
      
      // Make a note of the previous integrator value.
      float state_integrator_prev = state_integrator_;
      // Integrate the error of the state by using the trapezoid method with the stored value for the previous error.
      state_integrator_ = state_integrator_ + (Ts/2.0)*(error + state_error_);

      // Take the derivative of the error, using a dirty derivative with low pass filter value of tau.
      state_differentiator_ = (2.0*params.tau - Ts)/(2.0*params.tau + Ts)*state_differentiator_ + (2.0 /
                       (2.0*params.tau + Ts))*(error - state_error_);

      // Find the control efforts using the gains and calculated values.
      float up = params.state_kp*error;
      float ui = params.state_ki*state_integrator_;
      float ud = params.state_kd*rate; // If the rate is directly measured use it. Otherwise...
      // float ud = params.state_kd*state_differentiator;

      // Saturate the control effort between a defined max and min value.
      // Integrator anti-windup, if saturation occurs do not integrate that step.
      float control_effort = sat(up + ui + ud, max_value, min_value);
      float control_effort_unsat = up + ui + ud + phi_ff;
      if (fabs(control_effort - control_effort_unsat) < 0.0001)
      {
        state_integrator_ = state_integrator_prev;
      }

      // Save the error to use for integration and differentiation.
      // Then return the control effort.
      state_error_ = error;
      return control_effort;
    }
*/

float ControllerSucessiveLoop::course_hold(float chi_c, float chi, float phi_ff, float r)
{
  // For readability, declare parameters here that will be used in this function
  double frequency =
    params_.get_double("controller_output_frequency"); // Declared in controller_base
  double c_kp = params_.get_double("c_kp");
  double c_ki = params_.get_double("c_ki");
  double c_kd = params_.get_double("c_kd");
  double max_roll = params_.get_double("max_roll");

  double wrapped_chi_c = wrap_within_180(chi, chi_c);

  float error = wrapped_chi_c - chi;

  float Ts = 1.0 / frequency;

  float c_integrator_prev = c_integrator_;
  c_integrator_ = c_integrator_ + (Ts / 2.0) * (error + c_error_);

  float up = c_kp * error;
  float ui = c_ki * c_integrator_;
  float ud = c_kd * r;

  float phi_c = sat(up + ui + ud + phi_ff, max_roll * M_PI / 180.0, -max_roll * M_PI / 180.0);
  float phi_c_unsat = up + ui + ud + phi_ff;

  if (fabs(phi_c - phi_c_unsat) > 0.0001) { // If the controller is saturating don't integrate.
    c_integrator_ = c_integrator_prev;
  }

  c_error_ = error;
  return phi_c;
}

float ControllerSucessiveLoop::roll_hold(float phi_c, float phi, float p)
{
  // For readability, declare parameters here that will be used in this function
  double frequency =
    params_.get_double("controller_output_frequency"); // Declared in controller_base
  double r_kp = params_.get_double("r_kp");
  double r_ki = params_.get_double("r_ki");
  double r_kd = params_.get_double("r_kd");
  double max_a = params_.get_double("max_a");
  double trim_a = params_.get_double("trim_a");
  double pwm_rad_a = params_.get_double("pwm_rad_a"); // Declared in controller base

  float error = phi_c - phi;

  float Ts = 1.0 / frequency;

  float r_integrator_prev = r_integrator;
  r_integrator = r_integrator + (Ts / 2.0) * (error + r_error_);

  float up = r_kp * error;
  float ui = r_ki * r_integrator;
  float ud = r_kd * p;
  
  if (std::isnan(up)) {
    up = 0.0;
    RCLCPP_WARN(this->get_logger(), "Proportional control on the roll loop is NAN");
  }
  
  if (std::isnan(ui)) {
    r_integrator = 0.0;
    ui = 0.0;
    RCLCPP_WARN(this->get_logger(), "Integral control on the roll loop is NAN");
  }
  
  if (std::isnan(ud)) {
    ud = 0.0;
    RCLCPP_WARN(this->get_logger(), "Derivative control on the roll loop is NAN");
  }

  float delta_a = sat(trim_a / pwm_rad_a + up + ui - ud, max_a, -max_a);
  float delta_a_unsat = trim_a / pwm_rad_a + up + ui - ud;

  if (fabs(delta_a - delta_a_unsat) > 0.0001 && fabs(r_ki) > 0.00001) {
    r_integrator = r_integrator_prev;
  }

  r_error_ = error;
  return delta_a;
}

float ControllerSucessiveLoop::pitch_hold(float theta_c, float theta, float q)
{
  // For readability, declare parameters here that will be used in this function
  double frequency =
    params_.get_double("controller_output_frequency"); // Declared in controller_base
  double p_kp = params_.get_double("p_kp");
  double p_ki = params_.get_double("p_ki");
  double p_kd = params_.get_double("p_kd");
  double max_e = params_.get_double("max_e");
  double trim_e = params_.get_double("trim_e");
  double pwm_rad_e = params_.get_double("pwm_rad_e"); // Declared in controller_base

  float error = theta_c - theta;

  float Ts = 1.0 / frequency;

  float p_integrator_prev = p_integrator_;
  p_integrator_ = p_integrator_ + (Ts / 2.0) * (error + p_error_);

  float up = p_kp * error;
  float ui = p_ki * p_integrator_;
  float ud = p_kd * q;
  
  if (std::isnan(up)) {
    up = 0.0;
    RCLCPP_WARN(this->get_logger(), "Proportional control on the roll loop is NAN");
  }

  if (std::isnan(ui)) {
    p_integrator_ = 0.0;
    ui = 0.0;
    RCLCPP_WARN(this->get_logger(), "Integral control on the roll loop is NAN");
  }
  
  if (std::isnan(ud)) {
    ud = 0.0;
    RCLCPP_WARN(this->get_logger(), "Derivative control on the roll loop is NAN");
  }

  float delta_e = sat(trim_e / pwm_rad_e + up + ui - ud, max_e, -max_e);
  float delta_e_unsat = trim_e / pwm_rad_e + up + ui - ud;

  if (fabs(delta_e - delta_e_unsat) > 0.0001 && fabs(p_ki) > 0.00001) {
    p_integrator_ = p_integrator_prev;
  }

  p_error_ = error;
  return -delta_e; // TODO explain subtraction.
}

float ControllerSucessiveLoop::airspeed_with_throttle_hold(float va_c, float va)
{
  // For readability, declare parameters here that will be used in this function
  double frequency =
    params_.get_double("controller_output_frequency"); // Declared in controller_base
  double tau = params_.get_double("tau");
  double a_t_kp = params_.get_double("a_t_kp");
  double a_t_ki = params_.get_double("a_t_ki");
  double a_t_kd = params_.get_double("a_t_kd");
  double max_t = params_.get_double("max_t");
  double trim_t = params_.get_double("trim_t");

  float error = va_c - va;

  float Ts = 1.0 / frequency;

  float at_integrator_prev = at_integrator_;
  at_integrator_ = at_integrator_ + (Ts / 2.0) * (error + at_error_);
  at_differentiator_ = (2.0 * tau - Ts) / (2.0 * tau + Ts) * at_differentiator_
    + (2.0 / (2.0 * tau + Ts)) * (error - at_error_);

  float up = a_t_kp * error;
  float ui = a_t_ki * at_integrator_;
  float ud = a_t_kd * at_differentiator_;

  // Why isn't this one divided by the pwm_rad_t?
  float delta_t = sat(trim_t + up + ui + ud, max_t, 0);
  float delta_t_unsat = trim_t + up + ui + ud;

  if (fabs(delta_t - delta_t_unsat) > 0.0001) {
    at_integrator_ = at_integrator_prev;
  }

  at_error_ = error;
  return delta_t;
}

float ControllerSucessiveLoop::altitude_hold_control(float h_c, float h)
{
  // For readability, declare parameters here that will be used in this function
  double frequency =
    params_.get_double("controller_output_frequency"); // Declared in controller_base
  double alt_hz = params_.get_double("alt_hz");
  double tau = params_.get_double("tau");
  double a_kp = params_.get_double("a_kp");
  double a_ki = params_.get_double("a_ki");
  double a_kd = params_.get_double("a_kd");
  double max_pitch = params_.get_double("max_pitch");

  float error = h_c - h;

  float Ts = 1.0 / frequency;

  float a_integrator_prev = a_integrator_;
  if (-alt_hz + .01 < error && error < alt_hz - .01) {
    a_integrator_ = a_integrator_ + (Ts / 2.0) * (error + a_error_);
  } else {
    a_integrator_ = 0.0;
  }

  a_differentiator_ = (2.0 * tau - Ts) / (2.0 * tau + Ts) * a_differentiator_
    + (2.0 / (2.0 * tau + Ts)) * (error - a_error_);

  float up = a_kp * error;
  float ui = a_ki * a_integrator_;
  float ud = a_kd * a_differentiator_;

  float theta_c = sat(up + ui + ud, max_pitch * M_PI / 180.0, -max_pitch * M_PI / 180.0);
  float theta_c_unsat = up + ui + ud;
  if (fabs(theta_c - theta_c_unsat) > 0.0001) {
    a_integrator_ = a_integrator_prev;
  }

  a_error_ = error;
  return theta_c;
}

float ControllerSucessiveLoop::yaw_damper(float r)
{
  double frequency =
    params_.get_double("controller_output_frequency"); // Declared in controller_base
  float y_pwo = params_.get_double("y_pwo");
  float y_kr = params_.get_double("y_kr");
  float max_r = params_.get_double("max_r");

  float Ts = 1.0 / frequency;

  float n0 = 0.0;
  float n1 = 1.0;
  float d0 = y_pwo;
  float d1 = 1.0;

  float b0 = -y_kr * (2.0 * n1 - Ts * n0) / (2.0 * d1 + Ts * d0);
  float b1 = y_kr * (2.0 * n1 + Ts * n0) / (2.0 * d1 + Ts * d0);
  float a0 = y_kr * (2.0 * d1 - Ts * d0) / (2.0 * d1 + Ts * d0);

  float delta_r = -sat(a0 * delta_r_delay_ + b1 * r + b0 * r_delay_, max_r, -max_r);

  r_delay_ = r;
  delta_r_delay_ = delta_r;

  return delta_r;
}

//float controller_successive_loop::cooridinated_turn_hold(float v, const params_s &params)
//{
//    //todo finish this if you want...
//    return 0;
//}

// TODO: Add some error handling here.
float ControllerSucessiveLoop::sat(float value, float up_limit, float low_limit)
{
  // Set to upper limit if larger than that limit.
  // Set to lower limit if smaller than that limit.
  // Otherwise, do not change the value.

  if (up_limit < 0.0) {
    RCLCPP_WARN_ONCE(this->get_logger(),
                     "WARNING: Upper limit in saturation function is negative.");
  }

  float rVal;
  if (value > up_limit)
    rVal = up_limit;
  else if (value < low_limit)
    rVal = low_limit;
  else
    rVal = value;

  // Return the saturated value.
  return rVal;
}

float ControllerSucessiveLoop::adjust_h_c(float h_c, float h, float max_diff)
{
  double adjusted_h_c;

  // If the error in altitude is larger than the max altitude, adjust it to the max with the correct sign.
  // Otherwise, proceed as normal.
  if (abs(h_c - h) > max_diff) {
    adjusted_h_c = h + copysign(max_diff, h_c - h);
  } else {
    adjusted_h_c = h_c;
  }

  return adjusted_h_c;
}

void ControllerSucessiveLoop::declare_parameters()
{
  // Declare param with ROS2 and set the default value.
  params_.declare_bool("roll_command_override", false);
  params_.declare_bool("pitch_command_override", false);

  params_.declare_double("max_takeoff_throttle", 0.55);
  params_.declare_double("c_kp", 2.37);
  params_.declare_double("c_ki", .4);
  params_.declare_double("c_kd", .0);
  params_.declare_double("max_roll", 25.0);
  params_.declare_double("cmd_takeoff_pitch", 5.0);

  params_.declare_double("r_kp", .06);
  params_.declare_double("r_ki", .0);
  params_.declare_double("r_kd", .04);
  params_.declare_double("max_a", .15);
  params_.declare_double("max_r", 1.0);
  params_.declare_double("trim_a", 0.0);

  params_.declare_double("p_kp", -.15);
  params_.declare_double("p_ki", .0);
  params_.declare_double("p_kd", -.05);
  params_.declare_double("max_e", .15);
  params_.declare_double("max_pitch", 20.0);
  params_.declare_double("trim_e", 0.02);

  params_.declare_double("tau", 50.0);
  params_.declare_double("a_t_kp", .05);
  params_.declare_double("a_t_ki", .005);
  params_.declare_double("a_t_kd", 0.0);
  params_.declare_double("max_t", 1.0);
  params_.declare_double("trim_t", 0.5);

  params_.declare_double("a_kp", 0.015);
  params_.declare_double("a_ki", 0.003);
  params_.declare_double("a_kd", 0.0);

  params_.declare_double("y_pwo", .6349);
  params_.declare_double("y_kr", .85137);
}

} // namespace rosplane
