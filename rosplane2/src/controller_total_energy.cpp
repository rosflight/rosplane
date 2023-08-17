#include "controller_total_energy.hpp"

namespace rosplane2
{

controller_total_energy::controller_total_energy() : controller_successive_loop()
{
  // Initialize course hold, roll hold and pitch hold errors and integrators to zero.
  L_integrator_ = 0;
  E_integrator_ = 0;
}

void controller_total_energy::take_off_longitudinal_control(const struct params_s &params, const struct input_s &input, struct output_s &output)
{
  // Set throttle to not overshoot altitude.
  output.delta_t = sat(total_energy_throttle(input.Va_c, input.va, input.h_c, input.h, params, input.Ts), params.max_takeoff_throttle, 0);

  // Command a shallow pitch angle to gain altitude.
  output.theta_c = 5.0 * 3.14 / 180.0; //TODO add to params.
  output.delta_e = pitch_hold(output.theta_c, input.theta, input.q, params, input.Ts);
}

void controller_total_energy::take_off_exit()
{
  // Run parent exit code.
  controller_successive_loop::take_off_exit();
  
  L_integrator_ = 0;
  E_integrator_ = 0;

  // Place any controller code that should run as you exit the take-off regime here.
}

void controller_total_energy::climb_longitudinal_control(const struct params_s &params, const struct input_s &input, struct output_s &output)
{
  double adjusted_hc = adjust_h_c(input.h_c, input.h, params.alt_hz/2.0);

  // Find the control efforts for throttle and find the commanded pitch angle using total energy.
  output.delta_t = total_energy_throttle(input.Va_c, input.va, adjusted_hc, input.h, params, input.Ts);
  output.theta_c = total_energy_pitch(input.Va_c, input.va, adjusted_hc, input.h, params, input.Ts);
  output.delta_e = pitch_hold(output.theta_c, input.theta, input.q, params, input.Ts);
}

void controller_total_energy::climb_exit()
{
  //Run parent exit code
  controller_successive_loop::climb_exit();
  
  L_integrator_ = 0;
  E_integrator_ = 0;

  // Place any controller code that should run as you exit the climb regime here.
}

void controller_total_energy::alt_hold_longitudinal_control(const struct params_s &params, const struct input_s &input, struct output_s &output)
{
  // Saturate altitude command.
  double adjusted_hc = adjust_h_c(input.h_c, input.h, params.alt_hz);

  // Calculate the control effort to maintain airspeed and the required pitch angle to maintain altitude.
  output.delta_t = total_energy_throttle(input.Va_c, input.va, adjusted_hc, input.h, params, input.Ts);
  output.theta_c = total_energy_pitch(input.Va_c, input.va, adjusted_hc, input.h, params, input.Ts); // TODO remove capital from Va_c
  output.delta_e = pitch_hold(output.theta_c, input.theta, input.q, params, input.Ts);
}

void controller_total_energy::altitude_hold_exit()
{
  // Run parent exit code.
  controller_successive_loop::altitude_hold_exit();

  // Reset the integrators in the event of returning to the take-off regime (likely a crash).
  L_integrator_ = 0;
  E_integrator_ = 0;
}

float controller_total_energy::total_energy_throttle(float va_c, float va, float h_c, float h,
                                                     const struct params_s &params, float Ts)
{
  // Update energies based off of most recent data.
  update_energies(va_c, va, h_c, h, params);

  // Calculate total energy error, and normalize relative to the desired kinetic energy.
  float E_error = (K_error + U_error) / K_ref;

  // Integrate error.
  E_integrator_ = E_integrator_ + (Ts/2.0)*(E_error + E_error_prev_);

  E_error_prev_ = E_error;

  if (h < .5){
    E_integrator_ = 0;
  }

  // Return saturated throttle command.
  return sat(params.e_kp * E_error + params.e_ki * E_integrator_, params.max_t, 0.0) + params.trim_t;
}

float controller_total_energy::total_energy_pitch(float va_c, float va, float h_c, float h,
                                                  const struct params_s &params, float Ts)
{
  // Update energies based off of most recent data.
  update_energies(va_c, va, h_c, h, params);

  // Calculate energy balance error, and normalize relative to the desired kinetic energy.

  float L_error = (U_error - K_error) / K_ref;

  // Integrate error.
  L_integrator_ = L_integrator_ + (Ts/2.0)*(L_error + L_error_prev_);

  L_error_prev_ = L_error;

  // Return saturated pitch command.
  return sat(params.l_kp * L_error + params.l_ki * L_integrator_, 25.0 * M_PI/180.0, -25.0 * M_PI/180.0); // TODO remove hard coded bounds from all of rosplane!!!!
}

void controller_total_energy::update_energies(float va_c, float va, float h_c, float h, const struct params_s &params)
{

  // Calculate the error in kinetic energy.
  K_error = 0.5 * params.mass * (pow(va_c,2) - pow(va,2));

  // Clacualte the desired kinetic energy.
  K_ref = 0.5 * params.mass * pow(va_c,2);

  // Calculate the error in the potential energy.
  U_error = params.mass * params.gravity * sat(h_c - h, 5, -5); // TODO add limits to param
}

} //end namespace
