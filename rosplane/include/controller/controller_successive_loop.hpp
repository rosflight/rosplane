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
 * @file controller_successive_loop.hpp
 *
 * Implements the successive loop control described in Small Unmanned Aircraft by Tim McLain and Randy Beard
 *
 * @author Ian Reid <ian.young.reid@gmail.com>
 */

#ifndef CONTROLLER_SUCCESSIVE_H
#define CONTROLLER_SUCCESSIVE_H

#include <cmath>
#include "controller_state_machine.hpp"

namespace rosplane
{

class ControllerSucessiveLoop : public ControllerStateMachine
{
public:
  /**
   * Constructor to initialize node.
   */
  ControllerSucessiveLoop();

protected:
  /**
   * This function continually loops while the aircraft is in the take-off zone. The lateral and longitudinal control
   * for the take-off zone is called in this function.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void take_off(const Input & input, Output & output);

  /**
   * This function continually loops while the aircraft is in the climb zone. The lateral and longitudinal control
   * for the climb zone is called in this function.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void climb(const Input & input, Output & output);

  /**
   * This function continually loops while the aircraft is in the altitude hold zone. The lateral and longitudinal 
   * control for the altitude hold zone is called in this function.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void altitude_hold(const Input & input, Output & output);

  /**
   * This function runs when the aircraft exits the take-off zone. Any changes to the controller that need to happen
   * only once as the aircraft exits take-off mode should be placed here. This sets differentiators and integrators to 0.
   */
  virtual void take_off_exit();

  /**
   * This function runs when the aircraft exits the climb zone. Any changes to the controller that need to happen
   * only once as the aircraft exits climb mode should be placed here. This sets differentiators and integrators to 0.
   */
  virtual void climb_exit();

  /**
   * This function runs when the aircraft exits the altitude hold zone (usually a crash). Any changes to the controller that 
   * need to happen only once as the aircraft exits altitude mode should be placed here. This sets differentiators and
   * integrators to 0.
   */
  virtual void altitude_hold_exit();

  /**
   * This function runs the lateral control loops for the altitude hold zone.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void alt_hold_lateral_control(const Input & input, Output & output);

  /**
   * This function runs the longitudinal control loops for the altitude hold zone.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void alt_hold_longitudinal_control(const Input & input, Output & output);

  /**
   * This function runs the lateral control loops for the climb zone.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void climb_lateral_control(const Input & input, Output & output);

  /**
   * This function runs the longitudinal control loops for the climb zone.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void climb_longitudinal_control(const Input & input, Output & output);

  /**
   * This function runs the lateral control loops for the take-off zone.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void take_off_lateral_control(const Input & input, Output & output);

  /**
   * This function runs the longitudinal control loops for the take-off zone.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void take_off_longitudinal_control(const Input & input, Output & output);

  /**
   * The control loop for moving to and holding a commanded course.
   * @param chi_c The commanded course angle.
   * @param chi The current course angle.
   * @param phi_ff The roll angle feedforward term. This allows for faster convergence.
   * @param r The yaw rate taken from the gyro.
   * @return The commanded roll angle, to achieve the course angle.
   */
  float course_hold(float chi_c, float chi, float phi_ff, float r);

  /**
   * The difference between the commanded course angle and the current course angle.
   */
  float c_error_;

  /**
   * The integral of the error in course angle.
   */
  float c_integrator_;

  /**
   * The control loop for moving to and holding a commanded roll angle.
   * @param phi_c The commanded roll angle.
   * @param phi The current roll angle.
   * @param p The roll rate taken from the gyro.
   * @return The aileron deflection in radians required to achieve the commanded roll angle.
   */
  float roll_hold(float phi_c, float phi, float p);

  /**
   * The difference between the commanded roll angle and the current roll angle.
   */
  float r_error_;

  /**
   * The integral of the error in roll angle.
   */
  float r_integrator;

  /**
   * The control loop for moving to and holding a commanded pitch angle.
   * @param theta_c The commanded pitch angle.
   * @param theta The current pitch angle.
   * @param q The pitch rate taken from the gyro.
   * @return The elevator deflection in radians required to achieve the commanded pitch.
   */
  float pitch_hold(float theta_c, float theta, float q);

  /**
   * The difference between the commanded pitch angle and the current pitch angle.
   */
  float p_error_;

  /**
   * The integral of the error in pitch angle.
   */
  float p_integrator_;

  /**
   * The control loop that calculates the required throttle level to move to and maintain a commanded airspeed.
   * @param va_c The commanded airspeed.
   * @param va The current airspeed.
   * @return The required throttle between 0 (no throttle) and 1 (full throttle).
   */
  float airspeed_with_throttle_hold(float va_c, float va);

  /**
   * The difference between the commanded airspeed and the current airspeed.
   */
  float at_error_;

  /**
   * The integral of the error in airspeed.
   */
  float at_integrator_;

  /**
   * The derivative of the error in airspeed.
   */
  float at_differentiator_;

  /**
   * The control loop that calculates the required pitch angle to command to maintain a commanded altitude.
   * @param h_c The commanded altitude.
   * @param h The current altitude.
   * @return The commanded pitch angle to maintain and achieve the commanded altitude.
   */
  float altitude_hold_control(float h_c, float h);

  /**
   * The difference between the commanded altitude and the current altitude.
   */
  float a_error_;

  /**
   * The integral of the error in altitude.
   */
  float a_integrator_;

  /**
   * The derivative of the error in altitude.
   */
  float a_differentiator_;
  
  // These are not implemented here, though they are described in the UAV book.
  //    float cooridinated_turn_hold(float v, const struct params_s &params, float Ts); 
  //    float ct_error_;
  //    float ct_integrator_;
  //    float ct_differentiator_;
  
  /**
   *
   * This damps the adverse yaw by only allowing low frequency yaw movements.
   * If a high frequency yaw rate comes through it actively damps the values. 
   * This is described in the UAV book.
   * @param r The yaw rate of the aircraft.
   * @return The commanded ruddeer command.
   */
  float yaw_damper(float r);
  
  /**
   * The previous commanded rudder. This is used in the yaw damper.
   */
  float delta_r_delay_;

  /**
   * The previous yaw rate of the aircraft. This is used in the yaw damper
   */
  float r_delay_;

  /**
   * Saturate a given value to a maximum or minimum of the limits.
   * @param value The value to saturate.
   * @param up_limit The maximum the value can take on.
   * @param low_limit The minimum the value can take on.
   * @return The saturated value.
   */
  float sat(float value, float up_limit, float low_limit);

  /**
   * If the error in altitude is larger than the max altitude, adjust it to the max with the correct sign.
   * Otherwise, proceed as normal.
   * @param h_c The commaned altitiude.
   * @param h The current altitiude.
   * @param max_diff The maximum difference in the command and current (the max clamped error).
   * @return The adjusted altitude command.
   */
  float adjust_h_c(float h_c, float h, float max_diff);

private:

  /**
   * Declares the parameters associated to this controller, controller_successive_loop, so that ROS2 can see them.
   * Also declares default values before they are set to the values set in the launch script.
  */
  void declare_parameters();
};
} // namespace rosplane

#endif // CONTROLLER_SUCCESSIVE_H
