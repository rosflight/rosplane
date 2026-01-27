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
 * @file controller_ros.cpp
 * @author Ian Reid <ian.young.reid@gmail.com>
 */
#include "controller/controller_successive_loop.hpp"
#include "controller/controller_total_energy.hpp"
#include "controller/controller_ros.hpp"

namespace rosplane
{

ControllerROS::ControllerROS()
    : Node("controller")
    , params_(this)
    , params_initialized_(false)
{

  // Advertise published topics.
  actuators_pub_ = this->create_publisher<rosflight_msgs::msg::Command>("command", 10);
  controller_internals_pub_ =
    this->create_publisher<rosplane_msgs::msg::ControllerInternals>("controller_internals", 10);

  // Advertise subscribed topics and set bound callbacks.
  controller_commands_sub_ = this->create_subscription<rosplane_msgs::msg::ControllerCommands>(
    "controller_command", 10, std::bind(&ControllerROS::controller_commands_callback, this, _1));
  vehicle_state_sub_ = this->create_subscription<rosplane_msgs::msg::State>(
    "estimated_state", 10, std::bind(&ControllerROS::vehicle_state_callback, this, _1));

  // This flag indicates whether the first set of commands have been received.
  command_recieved_ = false;
  state_received_ = true;

  // Set the parameter callback, for when parameters are changed.
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ControllerROS::parametersCallback, this, std::placeholders::_1));

  // Declare the parameters for ROS2 param system.
  declare_parameters();
  // Set the values for the parameters, from the param file or use the deafault value.
  params_.set_parameters();

  params_initialized_ = true;

  set_timer();
}

void ControllerROS::declare_parameters()
{
  // Declare default parameters associated with this controller, controller_base
  params_.declare_double("pwm_rad_e", 1.0);
  params_.declare_double("pwm_rad_a", 1.0);
  params_.declare_double("pwm_rad_r", 1.0);
  params_.declare_double("controller_output_frequency", 100.0);
}

void ControllerROS::controller_commands_callback(
  const rosplane_msgs::msg::ControllerCommands::SharedPtr msg)
{

  // Set the flag that a command has been received.
  command_recieved_ = true;

  // Save the message to use in calculations.
  controller_commands_ = *msg;
}

void ControllerROS::vehicle_state_callback(const rosplane_msgs::msg::State::SharedPtr msg)
{
  // Set the flag that a state has been received.
  state_received_ = true;
  // Save the message to use in calculations.
  vehicle_state_ = *msg;
}

void ControllerROS::actuator_controls_publish()
{

  // Assemble inputs for the control algorithm.
  Input input;
  input.h = -vehicle_state_.p_d;
  input.va = vehicle_state_.va;
  input.phi = vehicle_state_.phi;
  input.theta = vehicle_state_.theta;
  input.chi = vehicle_state_.chi;
  input.p = vehicle_state_.p;
  input.q = vehicle_state_.q;
  input.r = vehicle_state_.r;
  input.va_c = controller_commands_.va_c;
  input.h_c = controller_commands_.h_c;
  input.chi_c = controller_commands_.chi_c;
  input.phi_ff = controller_commands_.phi_ff;

  Output output;

  // If a command was received, and a state received since last loop, begin control.
  if (command_recieved_ && state_received_) {
    state_received_ = false;

    // Control based off of inputs and parameters.
    control(input, output);

    // Convert control outputs to pwm.
    convert_to_pwm(output);

    rosflight_msgs::msg::Command actuators;

    // Find the current time, and save as a timestamp.
    rclcpp::Time now = this->get_clock()->now();

    // Attach the timestamp.
    actuators.header.stamp = now;

    // Do not ignore any of the actuators.
    actuators.ignore = 0;

    // Indicate that commands are for the actuators directly.
    actuators.mode = rosflight_msgs::msg::Command::MODE_PASS_THROUGH;

    // Package control efforts. If the output is infinite replace with 0.
    actuators.u[3] = (std::isfinite(output.delta_a)) ? output.delta_a : 0.0f;
    actuators.u[4] = (std::isfinite(output.delta_e)) ? output.delta_e : 0.0f;
    actuators.u[5] = (std::isfinite(output.delta_r)) ? output.delta_r : 0.0f;
    actuators.u[0] = (std::isfinite(output.delta_t)) ? output.delta_t : 0.0f;

    // Publish actuators.
    actuators_pub_->publish(actuators);

    // Publish the current control values
    rosplane_msgs::msg::ControllerInternals controller_internals;
    controller_internals.header.stamp = now;
    controller_internals.phi_c = output.phi_c;
    controller_internals.theta_c = output.theta_c;
    switch (output.current_zone) {
      case AltZones::TAKE_OFF:
        controller_internals.alt_zone = controller_internals.ZONE_TAKE_OFF;
        break;
      case AltZones::CLIMB:
        controller_internals.alt_zone = controller_internals.ZONE_CLIMB;
        break;
      case AltZones::ALTITUDE_HOLD:
        controller_internals.alt_zone = controller_internals.ZONE_ALTITUDE_HOLD;
        break;
      default:
        break;
    }
    controller_internals_pub_->publish(controller_internals);
  }
}

rcl_interfaces::msg::SetParametersResult
ControllerROS::parametersCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "One of the parameters given does not is not a parameter of the controller node.";

  bool success = params_.set_parameters_callback(parameters);
  if (success) {
    result.successful = true;
    result.reason = "success";
  }

  if (params_initialized_ && success) {
    std::chrono::microseconds curr_period = std::chrono::microseconds(
      static_cast<long long>(1.0 / params_.get_double("controller_output_frequency") * 1'000'000));
    if (timer_period_ != curr_period) {
      timer_->cancel();
      set_timer();
    }
  }

  return result;
}

void ControllerROS::set_timer()
{

  double frequency = params_.get_double("controller_output_frequency");
  timer_period_ = std::chrono::microseconds(static_cast<long long>(1.0 / frequency * 1'000'000));

  // Set timer to trigger bound callback (actuator_controls_publish) at the given periodicity.
  timer_ = rclcpp::create_timer(this, this->get_clock(), timer_period_,
                                   std::bind(&ControllerROS::actuator_controls_publish, this));
}

void ControllerROS::convert_to_pwm(Output & output)
{

  // Assign parameters from parameters object
  double pwm_rad_e = params_.get_double("pwm_rad_e");
  double pwm_rad_a = params_.get_double("pwm_rad_a");
  double pwm_rad_r = params_.get_double("pwm_rad_r");

  // Multiply each control effort (in radians) by a scaling factor to a pwm.
  output.delta_e = output.delta_e * pwm_rad_e;
  output.delta_a = output.delta_a * pwm_rad_a;
  output.delta_r = output.delta_r * pwm_rad_r;
}

} // namespace rosplane

int main(int argc, char * argv[])
{

  // Initialize ROS2 and then begin to spin control node.
  rclcpp::init(argc, argv);

  if (argc > 1) {
    if (strcmp(argv[1], "total_energy") == 0) {
      auto node = std::make_shared<rosplane::ControllerTotalEnergy>();
      RCLCPP_INFO_STREAM(node->get_logger(), "Using total energy control.");
      rclcpp::spin(node);
    } else {
      auto node = std::make_shared<rosplane::ControllerSucessiveLoop>();
      RCLCPP_INFO_STREAM(node->get_logger(), "Using default control.");
      rclcpp::spin(node);
    }
  } else {
    auto node = std::make_shared<rosplane::ControllerSucessiveLoop>();
    RCLCPP_INFO_STREAM(node->get_logger(), "Invalid control type or no type given, using default control.");
    rclcpp::spin(node);
  }

  return 0;
}
