#ifndef BUILD_CONTROLLER_STATE_MACHINE_H
#define BUILD_CONTROLLER_STATE_MACHINE_H

#include "controller_base.hpp"

namespace rosplane2{

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
  virtual void control(const struct params_s &params, const struct input_s &input, struct output_s &output);

protected:

  /**
   * The current part of the control algorithm based on altitude.
   */
  alt_zones current_zone;

  virtual void take_off(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;
  virtual void climb(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;
  virtual void altitude_hold(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

  virtual void take_off_exit() = 0;
  virtual void climb_exit() = 0;
  virtual void altitude_hold_exit() = 0;


};


} // end namespace

#endif //BUILD_CONTROLLER_STATE_MACHINE_H
