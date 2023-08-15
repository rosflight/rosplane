#include "controller_total_energy.hpp"

namespace rosplane2
{

controller_total_energy::controller_total_energy() : controller_successive_loop()
{
  L_integrator_ = 0;
  E_integrator_ = 0;

}

void controller_total_energy::take_off_longitudinal_control(const struct params_s &params, const struct input_s &input, struct output_s &output)
{
  // Set throttle to not overshoot altitude.
  output.delta_t = sat(total_energy_throttle(input.Va_c, input.va, input.h_c, input.h, params, input.Ts), params.max_takeoff_throttle, 0);

  // Command a shallow pitch angle to gain altitude.
  output.theta_c = 3.0 * 3.14 / 180.0;
  output.delta_e = pitch_hold(output.theta_c, input.theta, input.q, params, input.Ts);

}

void controller_total_energy::take_off_exit()
{
  controller_successive_loop::climb_exit();

// Place any controller code that should run as you exit the take-off regime here.
}

void controller_total_energy::climb_longitudinal_control(const struct params_s &params, const struct input_s &input, struct output_s &output)
{
  double adjusted_hc = adjust_h_c(input.h_c, input.h, params.alt_hz/2.0);

  // Find the control efforts for throttle and find the commanded pitch angle.
  output.delta_t = total_energy_throttle(input.Va_c, input.va, adjusted_hc, input.h, params, input.Ts);
  output.theta_c = total_energy_pitch(input.Va_c, input.va, adjusted_hc, input.h, params, input.Ts);
  output.delta_e = pitch_hold(output.theta_c, input.theta, input.q, params, input.Ts);
}

void controller_total_energy::climb_exit()
{
  controller_successive_loop::climb_exit();

// Place any controller code that should run as you exit the climb regime here.
}

void controller_total_energy::alt_hold_longitudinal_control(const struct params_s &params, const struct input_s &input, struct output_s &output){

  double adjusted_hc = adjust_h_c(input.h_c, input.h, params.alt_hz);

  // calculate the control effort to maintain airspeed and the required pitch angle to maintain altitude.
  output.delta_t = total_energy_throttle(input.Va_c, input.va, adjusted_hc, input.h, params, input.Ts);
  output.theta_c = total_energy_pitch(input.Va_c, input.va, adjusted_hc, input.h, params, input.Ts); // TODO remove capital from Va_c
  output.delta_e = pitch_hold(output.theta_c, input.theta, input.q, params, input.Ts);


}

void controller_total_energy::altitude_hold_exit()
{
  controller_successive_loop::altitude_hold_exit();

  // Reset the integrators in the event of returning to the take-off regime (likely a crash).

  L_integrator_ = 0;
  E_integrator_ = 0;
}

float controller_total_energy::total_energy_throttle(float va_c, float va, float h_c, float h,
                                                     const struct params_s &params, float Ts)
{
  float mass = 2.28; // TODO add to params.
  float gravity = 9.8; // TODO add to params.

  float K_error = 0.5 * mass * (pow(va_c,2) - pow(va,2));
  float K_ref = 0.5 * mass * pow(va_c,2);

  float U_error = mass * gravity * sat(h_c - h, -20, 20); // TODO add limits to param

  float E_error = (K_error + U_error) / K_ref;

  float E_kp = 5.0;

  float E_ki = .9;

  E_integrator_ = E_integrator_ + (Ts/2.0)*(E_error + E_error);

  return sat(E_kp * E_error + E_ki * E_integrator_, params.max_t, 0.0) + params.trim_t;
}

float controller_total_energy::total_energy_pitch(float va_c, float va, float h_c, float h,
                                                  const struct params_s &params,
                                                  float Ts) {
  float mass = 2.28; // TODO add to params.
  float gravity = 9.8; // TODO add to params.

  float K_error = 0.5 * mass * (pow(va_c,2) - pow(va,2));
  float K_ref = 0.5 * mass * pow(va_c,2);

  float U_error = mass * gravity * sat(h_c - h, -5, 5); // TODO add limits to param

  float L_error = (U_error - K_error) / K_ref;

  float L_kp = 5.0; // TODO add to params.

  float L_ki = 1.0; // TODO add to params.

  L_integrator_ = L_integrator_ + (Ts/2.0)*(L_error + L_error);

  return sat(L_kp * L_error + L_ki * L_integrator_, 25.0, 0.0); // TODO remove hard coded bounds from all of rosplane!!!!
}

} //end namespace
