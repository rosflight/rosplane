
#include "controller_state_machine.hpp"

namespace rosplane2 {

controller_state_machine::controller_state_machine() : controller_base() {

  // Initialize controller in take_off zone.
  current_zone = alt_zones::TAKE_OFF;

}

void controller_state_machine::control(const params_s &params, const input_s &input, output_s &output) {

  // This state machine changes the controls used based on the zone of flight path the aircraft is currently on.
  switch (current_zone) {
    case alt_zones::TAKE_OFF:

      take_off(params, input, output);

      // If the current altitude is outside the take-off zone (toz) then move to the climb state.
      if (input.h >= params.alt_toz) {

        take_off_exit();

        // Set zone to climb.
        RCLCPP_INFO(this->get_logger(), "climb");
        current_zone = alt_zones::CLIMB;
      }
      break;
    case alt_zones::CLIMB:

      climb(params, input, output);

      // Check to see if we have exited the climb zone.
      if (input.h >= input.h_c - params.alt_hz) {

        climb_exit();

        // Set the zone to altitude hold if we have enough altitude and reset errors, integrators and derivatives.
        RCLCPP_INFO(this->get_logger(), "hold");
        current_zone = alt_zones::ALTITUDE_HOLD;

      } else if (input.h <= params.alt_toz) {

        climb_exit();

        // Set to take off if too close to the ground.
        RCLCPP_INFO(this->get_logger(), "takeoff");
        current_zone = alt_zones::TAKE_OFF;
      }
      break;
    case alt_zones::ALTITUDE_HOLD:

      altitude_hold(params, input, output);

      // Check to see if you have gotten too close to the ground.
      if (input.h <= params.alt_toz) {

        altitude_hold_exit();

        // Set the control zone back to take off to regain altitude. and reset integral for course.
        RCLCPP_INFO(this->get_logger(), "take off");
        current_zone = alt_zones::TAKE_OFF;

      }
      break;
    default:
      break;
  }

  // Record current zone, to publish to controller internals.
  output.current_zone = current_zone;
}

} // end namespace