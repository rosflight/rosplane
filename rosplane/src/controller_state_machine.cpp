
#include "controller_state_machine.hpp"

namespace rosplane
{

controller_state_machine::controller_state_machine()
    : controller_base()
{

  // Initialize controller in take_off zone.
  current_zone = alt_zones::TAKE_OFF;

  // Declare parameters associated with this controller, controller_state_machine
  declare_parameters();

  // Set parameters according to the parameters in the launch file, otherwise use the default values
  params.set_parameters();
}

void controller_state_machine::control(const input_s & input,
                                       output_s & output)
{
  
  // For readability, declare parameters that will be used in this controller
  double alt_toz = params.get_double("alt_toz");
  double alt_hz = params.get_double("alt_hz");

  // This state machine changes the controls used based on the zone of flight path the aircraft is currently on.
  switch (current_zone) {
    case alt_zones::TAKE_OFF:

      // Run take-off controls.
      take_off(input, output);

      // If the current altitude is outside the take-off zone (toz) then move to the climb state.
      if (input.h >= alt_toz) {

        // Perform any exit tasks.
        take_off_exit();

        // Set zone to climb.
        RCLCPP_INFO(this->get_logger(), "climb");
        current_zone = alt_zones::CLIMB;
      }
      break;
    case alt_zones::CLIMB:

      // Run climb controls.
      climb(input, output);

      // Check to see if we have exited the climb zone.
      if (input.h >= input.h_c - alt_hz) {

        // Perform any exit tasks.
        climb_exit();

        // Set the zone to altitude hold if we have enough altitude and reset errors, integrators and derivatives.
        RCLCPP_INFO(this->get_logger(), "hold");
        current_zone = alt_zones::ALTITUDE_HOLD;

      } else if (input.h <= alt_toz) {

        // Perform any exit tasks.
        climb_exit();

        // Set to take off if too close to the ground.
        RCLCPP_INFO(this->get_logger(), "takeoff");
        current_zone = alt_zones::TAKE_OFF;
      }
      break;
    case alt_zones::ALTITUDE_HOLD:

      // Run altitude hold controls.
      altitude_hold(input, output);

      // Check to see if you have gotten too close to the ground.
      if (input.h <= alt_toz) {

        // Perform any exit tasks.
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

void controller_state_machine::declare_parameters()
{
  // Declare param with ROS2 and set the default value.
  params.declare_double("alt_toz", 5.0);
  params.declare_double("alt_hz", 10.0);
}

} // namespace rosplane