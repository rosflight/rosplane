#ifndef BUILD_CONTROLLER_STATE_MACHINE_H
#define BUILD_CONTROLLER_STATE_MACHINE_H

#include "controller_base.hpp"

namespace rosplane
{

class controller_state_machine : public controller_base
{

public:
  controller_state_machine();

  /**
 * The state machine for the control algorithm for the autopilot.
 * @param params The parameters that define the algorithm such as control gains.
 * @param input The command inputs to the controller such as course and airspeed.
 * @param output The control efforts calculated and selected intermediate values.
 */
  virtual void control(const struct params_s & params, const struct input_s & input,
                       struct output_s & output);

protected:
  /**
   * The current zone of the control algorithm based on altitude.
   */
  alt_zones current_zone;

  /**
   * This function continually loops while the aircraft is in the take-off zone. It is implemented by the child.
   * @param params The parameters that define the algorithm such as control gains.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void take_off(const struct params_s & params, const struct input_s & input,
                        struct output_s & output) = 0;

  /**
   * This function continually loops while the aircraft is in the climb zone. It is implemented by the child.
   * @param params The parameters that define the algorithm such as control gains.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void climb(const struct params_s & params, const struct input_s & input,
                     struct output_s & output) = 0;

  /**
   * This function continually loops while the aircraft is in the altitude hold zone. It is implemented by the child.
   * @param params The parameters that define the algorithm such as control gains.
   * @param input The command inputs to the controller such as course and airspeed.
   * @param output The control efforts calculated and selected intermediate values.
   */
  virtual void altitude_hold(const struct params_s & params, const struct input_s & input,
                             struct output_s & output) = 0;

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
};

} // namespace rosplane

#endif //BUILD_CONTROLLER_STATE_MACHINE_H
