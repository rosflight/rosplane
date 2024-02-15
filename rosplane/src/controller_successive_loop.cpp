#include "controller_successive_loop.hpp"

#include "iostream"
#include <rclcpp/logging.hpp>

namespace rosplane
{


double wrap_within_180(double fixed_heading, double wrapped_heading) {
    // wrapped_heading - number_of_times_to_wrap * 2pi
    return wrapped_heading - floor((wrapped_heading - fixed_heading) / (2 * M_PI) + 0.5) * 2 * M_PI;
}

controller_successive_loop::controller_successive_loop() : controller_state_machine()
{
  // Initialize course hold, roll hold and pitch hold errors and integrators to zero.
  c_error_ = 0;
  c_integrator_ = 0;
  r_error_ = 0;
  r_integrator = 0;
  p_error_ = 0;
  p_integrator_ = 0;
}

void controller_successive_loop::take_off(const struct params_s &params, const struct input_s &input, struct output_s &output)
{
  // Run lateral and longitudinal controls.
  take_off_lateral_control(params, input, output);
  take_off_longitudinal_control(params, input, output);
}

void controller_successive_loop::take_off_exit()
{
  // Put any code that should run as the airplane exits take off mode.
}

void controller_successive_loop::climb(const struct params_s &params, const struct input_s &input, struct output_s &output)
{
  // Run lateral and longitudinal controls.
  climb_lateral_control(params, input, output);
  climb_longitudinal_control(params, input, output);
}

void controller_successive_loop::climb_exit()
{
  // Reset differentiators, integrators and errors.
  at_error_ = 0;
  at_integrator_ = 0;
  at_differentiator_ = 0;
  a_error_ = 0;
  a_integrator_ = 0;
  a_differentiator_ = 0;
}

void controller_successive_loop::altitude_hold(const struct params_s &params, const struct input_s &input, struct output_s &output)
{
  // Run lateral and longitudinal controls.
  alt_hold_lateral_control(params, input, output);
  alt_hold_longitudinal_control(params, input, output);
}

void controller_successive_loop::altitude_hold_exit()
{
  // Reset integrators.
  c_integrator_ = 0;
}

void controller_successive_loop::alt_hold_lateral_control(const struct params_s &params, const struct input_s &input, struct output_s &output)
{
  // Set rudder command to zero, can use cooridinated_turn_hold if implemented.
  // Find commanded roll angle in order to achieve commanded course.
  // Find aileron deflection required to acheive required roll angle.
  output.delta_r = 0; //cooridinated_turn_hold(input.beta, params)
  output.phi_c = course_hold(input.chi_c, input.chi, input.phi_ff, input.r, params);

  if (params.roll_tuning_debug_override){
    output.phi_c = tuning_debug_override_msg_.phi_c;
  }

  output.delta_a = roll_hold(output.phi_c, input.phi, input.p, params);
}

void controller_successive_loop::alt_hold_longitudinal_control(const struct params_s &params, const struct input_s &input, struct output_s &output)
{
  // Saturate the altitude command.
  double adjusted_hc = adjust_h_c(input.h_c, input.h, params.alt_hz);

  // Control airspeed with throttle loop and altitude with commanded pitch and drive aircraft to commanded pitch.
  output.delta_t = airspeed_with_throttle_hold(input.Va_c, input.va, params);
  output.theta_c = altitude_hold_control(adjusted_hc, input.h, params);

  if (params.pitch_tuning_debug_override){
    output.theta_c = tuning_debug_override_msg_.theta_c;
  }

  output.delta_e = pitch_hold(output.theta_c, input.theta, input.q, params);
}

void controller_successive_loop::climb_lateral_control(const struct params_s &params, const struct input_s &input, struct output_s &output)
{
  // Maintain straight flight while gaining altitude.
  output.phi_c = 0;
  output.delta_a = roll_hold(output.phi_c, input.phi, input.p, params);
  output.delta_r = 0;
}

void controller_successive_loop::climb_longitudinal_control(const struct params_s &params, const struct input_s &input, struct output_s &output)
{
  // Saturate the altitude command.
  double adjusted_hc = adjust_h_c(input.h_c, input.h, params.alt_hz);

  // Find the control efforts for throttle and find the commanded pitch angle.
  output.delta_t = airspeed_with_throttle_hold(input.Va_c, input.va, params);
  output.theta_c = altitude_hold_control(adjusted_hc, input.h, params);
  output.delta_e = pitch_hold(output.theta_c, input.theta, input.q, params);
}

void controller_successive_loop::take_off_lateral_control(const struct params_s &params, const struct input_s &input, struct output_s &output)
{
  // In the take-off zone maintain level straight flight by commanding a roll angle of 0 and rudder of 0.
  output.delta_r = 0;
  output.phi_c = 0;
  output.delta_a = roll_hold(output.phi_c, input.phi, input.p, params);
}

void controller_successive_loop::take_off_longitudinal_control(const struct params_s &params, const struct input_s &input, struct output_s &output)
{
  // Set throttle to not overshoot altitude.
  output.delta_t = sat(airspeed_with_throttle_hold(input.Va_c, input.va, params), params.max_takeoff_throttle, 0);

  // Command a shallow pitch angle to gain altitude.
  output.theta_c = 5.0 * 3.14 / 180.0; // TODO add to params.
  output.delta_e = pitch_hold(output.theta_c, input.theta, input.q, params);
}

/// All the following control loops follow this basic outline.
/*
    float controller_successive_loop::pid_control(float command_val, float actual_val, float rate, // Not all loops use rate.
                                          const params_s &params)
    {
      // Find the error between the commanded and actual value.
      float error = commanded_val - actual_val;

      float Ts = 1.0/params.frequency;

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
      // If the saturation occurs, and you are using integral control, adjust the integrator.
      float control_effort = sat(up + ui + ud, max_value, min_value);
      if (fabs(params.c_ki) >= 0.00001)
      {
        float control_effort_unsat = up + ui + ud + phi_ff;
        state_integrator_ = state_integrator_ + (Ts/params.state_ki)*(control_effort - control_effort_unsat);
      }

      // Save the error to use for integration and differentiation.
      // Then return the control effort.
      state_error_ = error;
      return control_effort;
    }
*/

float controller_successive_loop::course_hold(float chi_c, float chi, float phi_ff, float r, const params_s &params)
{

  double wrapped_chi_c = wrap_within_180(chi, chi_c);
  

  float error = wrapped_chi_c - chi;

  float Ts = 1.0/params.frequency;

  c_integrator_ = c_integrator_ + (Ts/2.0)*(error + c_error_);

  float up = params.c_kp*error;
  float ui = params.c_ki*c_integrator_;
  float ud = params.c_kd*r;

  float phi_c = sat(up + ui + ud + phi_ff, params.max_roll*3.14/180.0, -params.max_roll*3.14/180.0);
  if (fabs(params.c_ki) >= 0.00001)
  {
    float phi_c_unsat = up + ui + ud + phi_ff;
    c_integrator_ = c_integrator_ + (Ts/params.c_ki)*(phi_c - phi_c_unsat);
  }

  c_error_ = error;
  return phi_c;
}

float controller_successive_loop::roll_hold(float phi_c, float phi, float p, const params_s &params)
{
  float error = phi_c - phi;

  float Ts = 1.0/params.frequency;

  r_integrator = r_integrator + (Ts/2.0)*(error + r_error_);

  float up = params.r_kp*error;
  float ui = params.r_ki*r_integrator;
  float ud = params.r_kd*p;

  float delta_a = sat(up + ui - ud, params.max_a, -params.max_a);
  if (fabs(params.r_ki) >= 0.00001)
  {
    float delta_a_unsat = up + ui - ud;
    r_integrator = r_integrator + (Ts/params.r_ki)*(delta_a - delta_a_unsat);
  }

  r_error_ = error;
  return delta_a;
}

float controller_successive_loop::pitch_hold(float theta_c, float theta, float q, const params_s &params)
{
  float error = theta_c - theta;

  float Ts = 1.0/params.frequency;

  p_integrator_ = p_integrator_ + (Ts/2.0)*(error + p_error_);

  float up = params.p_kp*error;
  float ui = params.p_ki*p_integrator_;
  float ud = params.p_kd*q;


  float delta_e = sat(params.trim_e/params.pwm_rad_e + up + ui - ud, params.max_e, -params.max_e);


  if (fabs(params.p_ki) >= 0.00001)
  {
    float delta_e_unsat = params.trim_e/params.pwm_rad_e + up + ui - ud;
    p_integrator_ = p_integrator_ + (Ts/params.p_ki)*(delta_e - delta_e_unsat);
  }

  p_error_ = error;
  return -delta_e; // TODO explain subtraction.
}

float controller_successive_loop::airspeed_with_throttle_hold(float Va_c, float Va, const params_s &params)
{
  float error = Va_c - Va;

  float Ts = 1.0/params.frequency;

  at_integrator_ = at_integrator_ + (Ts/2.0)*(error + at_error_);
  at_differentiator_ = (2.0*params.tau - Ts)/(2.0*params.tau + Ts)*at_differentiator_ + (2.0 /
                       (2.0*params.tau + Ts))*(error - at_error_);

  float up = params.a_t_kp*error;
  float ui = params.a_t_ki*at_integrator_;
  float ud = params.a_t_kd*at_differentiator_;

  float delta_t = sat(params.trim_t + up + ui + ud, params.max_t, 0);
  if (fabs(params.a_t_ki) >= 0.00001)
  {
    float delta_t_unsat = params.trim_t + up + ui + ud;
    at_integrator_ = at_integrator_ + (Ts/params.a_t_ki)*(delta_t - delta_t_unsat);
  }

  at_error_ = error;
  return delta_t;
}

float controller_successive_loop::altitude_hold_control(float h_c, float h, const params_s &params)
{
  float error = h_c - h;

  float Ts = 1.0/params.frequency;

  if (-params.alt_hz + .01 < error && error < params.alt_hz - .01) {
    a_integrator_ = a_integrator_ + (Ts / 2.0) * (error + a_error_);
  }
  else{
    a_integrator_ = 0.0;
  }

  a_differentiator_ = (2.0*params.tau - Ts)/(2.0*params.tau + Ts)*a_differentiator_ + (2.0 /
                      (2.0*params.tau + Ts))*(error - a_error_);

  float up = params.a_kp*error;
  float ui = params.a_ki*a_integrator_;
  float ud = params.a_kd*a_differentiator_;

  float theta_c = sat(up + ui + ud, 20.0*3.14/180.0, -20.0*3.14/180.0);
  if (fabs(params.a_ki) >= 0.00001)
  {
    float theta_c_unsat = up + ui + ud;
    a_integrator_ = a_integrator_ + (Ts/params.a_ki)*(theta_c - theta_c_unsat);
  }


  a_error_ = error;
  return theta_c;
}

//float controller_successive_loop::cooridinated_turn_hold(float v, const params_s &params, float Ts)
//{
//    //todo finish this if you want...
//    return 0;
//}

float controller_successive_loop::sat(float value, float up_limit, float low_limit)
{
  // Set to upper limit if larger than that limit.
  // Set to lower limit if smaller than that limit.
  // Otherwise, do not change the value.
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

float controller_successive_loop::adjust_h_c(float h_c, float h, float max_diff) {
  double adjusted_h_c;

  // If the error in altitude is larger than the max altitude, adjust it to the max with the correct sign.
  // Otherwise, proceed as normal.
  if (abs(h_c - h) > max_diff){
    adjusted_h_c = h + copysign(max_diff, h_c - h);
  }
  else{
    adjusted_h_c = h_c;
  }

  return adjusted_h_c;
}

} //end namespace
