/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2025 Ian Reid, BYU MAGICC Lab.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file controller_total_energy.hpp
 *
 * Implements the total energy control described in Small Unmanned Aircraft by Tim McLain and Randy Beard
 *
 * @author Ian Reid <ian.young.reid@gmail.com>
 */

#ifndef BUILD_CONTROLLER_TOTAL_ENERGY_H
#define BUILD_CONTROLLER_TOTAL_ENERGY_H

#include "controller_successive_loop.hpp"

namespace rosplane
{

class ControllerTotalEnergy : public ControllerSucessiveLoop
{
public:
  /**
   * Constructor to initialize node.
   */
  ControllerTotalEnergy();

protected:
  /**
   * This function overrides the longitudinal control loops for the take-off zone.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void take_off_longitudinal_control(const Input & input, Output & output);

  /**
   * This function overrides the longitudinal control loops for the climb zone.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void climb_longitudinal_control(const Input & input, Output & output);

  /**
   * This function overrides the longitudinal control loops for the altitude hold zone.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void alt_hold_longitudinal_control(const Input & input, Output & output);

  /**
   * This function overrides when the aircraft exits the take-off zone. Any changes to the controller that need to happen
   * only once as the aircraft exits take-off mode should be placed here. This sets differentiators and integrators to 0.
   */
  virtual void take_off_exit();

  /**
   * This function overrides when the aircraft exits the climb zone. Any changes to the controller that need to happen
   * only once as the aircraft exits climb mode should be placed here. This sets differentiators and integrators to 0.
   */
  virtual void climb_exit();

  /**
   * This function overrides when the aircraft exits the altitude hold zone (usually a crash). Any changes to the controller that 
   * need to happen only once as the aircraft exits altitude mode should be placed here. This sets differentiators and
   * integrators to 0.
   */
  virtual void altitude_hold_exit();

  /**
   * This uses the error in total energy to find the necessary throttle to acheive that energy.
   * @param va_c This is the commanded airspeed.
   * @param va This is the actual airspeed.
   * @param h_c This is the commanded altitude.
   * @param h This is the actual altitude.
   * @param Ts The sampling period in seconds.
   * @return The throttle value saturated between 0 and the parameter of max throttle.
   */
  float total_energy_throttle(float va_c, float va, float h_c, float h);

  /**
   * This uses the error in the balance of energy to find the necessary elevator deflection to acheive that energy.
   * @param va_c This is the commanded airspeed.
   * @param va This is the actual airspeed.
   * @param h_c This is the commanded altitude.
   * @param h This is the actual altitude.
   * @param Ts The sampling period in seconds.
   * @return The pitch command value saturated between min and max pitch.
   */
  float total_energy_pitch(float va_c, float va, float h_c, float h);

  /**
   * This calculates and updates the kinetic energy reference and error, the potential energy error.
   * @param va_c This is the commanded airspeed.
   * @param va This is the actual airspeed.
   * @param h_c This is the commanded altitude.
   * @param h This is the actual altitude.
   */
  void update_energies(float va_c, float va, float h_c, float h);

  /**
   * This is the integral value for the error in the total energy.
   */
  float E_integrator_;

  /**
   * This is the integral value for the error in the balance of energy.
   */
  float L_integrator_;

  /**
   * This is the current reference (desired) kinetic energy.
   */
  float K_ref_;

  /**
   * This is the current error in the kinetic energy.
   */
  float K_error_;

  /**
   * This is the current error in the potential energy.
   */
  float U_error_;

  /**
   * The previous error in the energy balance.
   */
  float L_error_prev_;

  /**
   * The previous error in the total energy.
   */
  float E_error_prev_;

private:

  /**
   * Declares the parameters associated to this controller, controller_successive_loop, so that ROS2 can see them.
   * Also declares default values before they are set to the values set in the launch script.
  */
  void declare_parameters();
};
} // namespace rosplane

#endif //BUILD_CONTROLLER_TOTAL_ENERGY_H
