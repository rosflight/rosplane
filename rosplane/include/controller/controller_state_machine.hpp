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
 * @file controller_state_machine.hpp
 *
 * Implements the state machine for controlling a fixedwing.
 *
 * @author Ian Reid <ian.young.reid@gmail.com>
 */

#ifndef BUILD_CONTROLLER_STATE_MACHINE_H
#define BUILD_CONTROLLER_STATE_MACHINE_H

#include "controller_ros.hpp"

namespace rosplane
{

class ControllerStateMachine : public ControllerROS
{

public:
  ControllerStateMachine();

  /**
 * The state machine for the control algorithm for the autopilot.
 * @param input The command inputs to the controller such as course and airspeed.
 * @param output The control efforts calculated and selected intermediate values.
 */
  virtual void control(const Input & input, Output & output);

protected:
  /**
   * The current zone of the control algorithm based on altitude.
   */
  AltZones current_zone_;

  /**
   * This function continually loops while the aircraft is in the take-off zone. It is implemented by the child.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void take_off(const Input & input, Output & output) = 0;

  /**
   * This function continually loops while the aircraft is in the climb zone. It is implemented by the child.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void climb(const Input & input, Output & output) = 0;

  /**
   * This function continually loops while the aircraft is in the altitude hold zone. It is implemented by the child.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void altitude_hold(const Input & input, Output & output) = 0;

  /**
   * This function runs when the aircraft exits the take-off zone this is often used to reset integrator values. It is
   * implemented by the child.
   */
  virtual void take_off_exit() = 0;

  /**
   * This function runs when the aircraft exits the climb zone this is often used to reset integrator values. It is
   * implemented by the child.
   */
  virtual void climb_exit() = 0;

  /**
   * This function runs when the aircraft exits the altitude hold zone this is often used to reset integrator values.
   * It is implemented by the child.
   */
  virtual void altitude_hold_exit() = 0;

private:
  /**
   * Declares the parameters associated to this controller, controller_state_machine, so that ROS2 can see them.
   * Also declares default values before they are set to the values set in the launch script.
  */
  void declare_parameters();
};

} // namespace rosplane

#endif //BUILD_CONTROLLER_STATE_MACHINE_H
