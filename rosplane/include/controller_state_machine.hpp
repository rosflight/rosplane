#ifndef BUILD_CONTROLLER_STATE_MACHINE_H
#define BUILD_CONTROLLER_STATE_MACHINE_H

#include "controller_base.hpp"

namespace rosplane
{

class ControllerStateMachine : public ControllerBase
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
