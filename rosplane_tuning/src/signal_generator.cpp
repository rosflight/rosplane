/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2023 Brandon Sutherland, BYU MAGICC Lab.
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
 * @file tuning_signal_generator.hpp
 *
 * @author Brandon Sutherland <brandonsutherland2@gmail.com>
 */

#include <chrono>
#include <cmath>
#include <string>

#include "signal_generator.hpp"

namespace rosplane
{
TuningSignalGenerator::TuningSignalGenerator()
    : Node("signal_generator")
    , controller_output_(ControllerOutput::ROLL)
    , signal_type_(SignalType::SQUARE)
    , publish_rate_hz_(0)
    , signal_magnitude_(0)
    , frequency_hz_(0)
    , initial_time_(0)
    , is_paused_(true)
    , paused_time_(0)
    , single_period_start_time_(0)
{
  this->declare_parameter("controller_output", "roll");
  this->declare_parameter("signal_type", "step");
  this->declare_parameter("publish_rate_hz", 100.0);
  this->declare_parameter("signal_magnitude", 1.0);
  this->declare_parameter("frequency_hz", 0.2);
  this->declare_parameter("default_va_c", 15.0);
  this->declare_parameter("default_h_c", 40.0);
  this->declare_parameter("default_chi_c", 0.0);
  this->declare_parameter("default_theta_c", 0.0);
  this->declare_parameter("default_phi_c", 0.0);

  update_params();
  initial_time_ = this->get_clock()->now().seconds();

  command_publisher_ =
    this->create_publisher<rosplane_msgs::msg::ControllerCommands>("/controller_command", 1);

  publish_timer_ =
    this->create_wall_timer(std::chrono::milliseconds(static_cast<long>(1000 / publish_rate_hz_)),
                            std::bind(&TuningSignalGenerator::publish_timer_callback, this));

  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&TuningSignalGenerator::param_callback, this, std::placeholders::_1));

  step_toggle_service_ = this->create_service<std_srvs::srv::Trigger>(
    "toggle_step_signal",
    std::bind(&TuningSignalGenerator::step_toggle_service_callback, this, std::placeholders::_1,
              std::placeholders::_2));
  reset_service_ = this->create_service<std_srvs::srv::Trigger>(
    "reset_signal",
    std::bind(&TuningSignalGenerator::reset_service_callback, this, std::placeholders::_1,
              std::placeholders::_2));
  pause_service_ = this->create_service<std_srvs::srv::Trigger>(
    "pause_signal",
    std::bind(&TuningSignalGenerator::pause_service_callback, this, std::placeholders::_1,
              std::placeholders::_2));
  start_continuous_service_ = this->create_service<std_srvs::srv::Trigger>(
    "start_continuous_signal",
    std::bind(&TuningSignalGenerator::start_continuous_service_callback, this,
              std::placeholders::_1, std::placeholders::_2));
  start_single_service_ = this->create_service<std_srvs::srv::Trigger>(
    "start_single_period_signal",
    std::bind(&TuningSignalGenerator::start_single_service_callback, this, std::placeholders::_1,
              std::placeholders::_2));
}

void TuningSignalGenerator::publish_timer_callback()
{
  update_params();

  double elapsed_time = this->get_clock()->now().seconds() - initial_time_ - paused_time_;

  // Check if only suppose to run for single period, pausing when complete
  if (abs(single_period_start_time_) > 0.01
      && (single_period_start_time_ + (1 / frequency_hz_)) <= this->get_clock()->now().seconds()) {
    single_period_start_time_ = 0;
    is_paused_ = true;
  }

  // Check if step toggle needs to be reset
  if (signal_type_ != SignalType::STEP) {
    step_toggled_ = false;
  }

  // If paused, negate passing of time but keep publishing
  if (is_paused_) {
    paused_time_ += 1 / publish_rate_hz_;
  }

  // Get value for signal
  double amplitude = signal_magnitude_ / 2;
  double center_value = 0;
  switch (controller_output_) {
    case ControllerOutput::ROLL:
      center_value = default_phi_c_;
      break;
    case ControllerOutput::PITCH:
      center_value = default_theta_c_;
      break;
    case ControllerOutput::ALTITUDE:
      center_value = default_h_c_;
      break;
    case ControllerOutput::COURSE:
      center_value = default_chi_c_;
      break;
    case ControllerOutput::AIRSPEED:
      center_value = default_va_c_;
      break;
  }
  center_value += amplitude;
  double signal_value = 0;
  switch (signal_type_) {
    case SignalType::STEP:
      signal_value = get_step_signal(step_toggled_, amplitude, center_value);
      break;
    case SignalType::SQUARE:
      signal_value = get_square_signal(elapsed_time, amplitude, frequency_hz_, center_value);
      break;
    case SignalType::SAWTOOTH:
      signal_value = get_sawtooth_signal(elapsed_time, amplitude, frequency_hz_, center_value);
      break;
    case SignalType::TRIANGLE:
      signal_value = get_triangle_signal(elapsed_time, amplitude, frequency_hz_, center_value);
      break;
    case SignalType::SINE:
      signal_value = get_sine_signal(elapsed_time, amplitude, frequency_hz_, center_value);
      break;
  }

  // Creates message with default values
  rosplane_msgs::msg::ControllerCommands command_message;
  command_message.header.stamp = this->get_clock()->now();
  command_message.va_c = default_va_c_;
  command_message.h_c = default_h_c_;
  command_message.chi_c = default_chi_c_;
  command_message.theta_c = default_theta_c_;
  command_message.phi_c = default_phi_c_;

  // Publish message
  switch (controller_output_) {
    case ControllerOutput::ROLL:
      command_message.phi_c = signal_value;
      break;
    case ControllerOutput::PITCH:
      command_message.theta_c = signal_value;
      break;
    case ControllerOutput::ALTITUDE:
      command_message.h_c = signal_value;
      break;
    case ControllerOutput::COURSE:
      command_message.chi_c = signal_value;
      break;
    case ControllerOutput::AIRSPEED:
      command_message.va_c = signal_value;
      break;
  }
  command_publisher_->publish(command_message);
}

rcl_interfaces::msg::SetParametersResult
TuningSignalGenerator::param_callback(const std::vector<rclcpp::Parameter> & params)
{
  for (const auto & param : params) {
    if (param.get_name() == "controller_output" || param.get_name() == "signal_type") {
      reset();
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

bool TuningSignalGenerator::step_toggle_service_callback(
  const std_srvs::srv::Trigger::Request::SharedPtr & req,
  const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  if (signal_type_ != SignalType::STEP) {
    res->success = false;
    res->message = "Service valid only for step type signal";
    return true;
  }

  if (step_toggled_) {
    step_toggled_ = false;
  } else {
    step_toggled_ = true;
  }

  res->success = true;
  return true;
}

bool TuningSignalGenerator::reset_service_callback(
  const std_srvs::srv::Trigger::Request::SharedPtr & req,
  const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  reset();
  res->success = true;
  return true;
}

bool TuningSignalGenerator::pause_service_callback(
  const std_srvs::srv::Trigger::Request::SharedPtr & req,
  const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  if (signal_type_ == SignalType::STEP) {
    res->success = false;
    res->message = "Service not valid for step type signal";
    return true;
  }

  is_paused_ = true;
  single_period_start_time_ = 0;

  res->success = true;
  return true;
}

bool TuningSignalGenerator::start_continuous_service_callback(
  const std_srvs::srv::Trigger::Request::SharedPtr & req,
  const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  if (signal_type_ == SignalType::STEP) {
    res->success = false;
    res->message = "Service not valid for step type signal";
    return true;
  }

  is_paused_ = false;
  single_period_start_time_ = 0;

  res->success = true;
  return true;
}

bool TuningSignalGenerator::start_single_service_callback(
  const std_srvs::srv::Trigger::Request::SharedPtr & req,
  const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  if (signal_type_ == SignalType::STEP) {
    res->success = false;
    res->message = "Service not valid for step type signal";
    return true;
  }

  is_paused_ = false;
  single_period_start_time_ = this->get_clock()->now().seconds();

  res->success = true;
  return true;
}

double TuningSignalGenerator::get_step_signal(bool step_toggled, double amplitude,
                                              double center_value)
{
  return (step_toggled - 0.5) * 2 * amplitude + center_value;
}

double TuningSignalGenerator::get_square_signal(double elapsed_time, double amplitude,
                                                double frequency, double center_value)
{
  // amplitude * (1 to -1 switching value) + center value
  return amplitude * ((static_cast<int>(elapsed_time * frequency * 2) % 2) * 2 - 1) + center_value;
}

double TuningSignalGenerator::get_sawtooth_signal(double elapsed_time, double amplitude,
                                                  double frequency, double center_value)
{
  // slope * elapsed_time - num_cycles * offset_per_cycle + center value
  return 2 * amplitude
    * (elapsed_time * frequency - static_cast<int>(elapsed_time * frequency) - 0.5)
    + center_value;
}

double TuningSignalGenerator::get_triangle_signal(double elapsed_time, double amplitude,
                                                  double frequency, double center_value)
{
  // (1 to -1 switching value) * sawtooth_at_twice_the_rate + center_value
  return -((static_cast<int>(elapsed_time * frequency * 2) % 2) * 2 - 1) * 2 * amplitude
    * (2 * elapsed_time * frequency - static_cast<int>(2 * elapsed_time * frequency) - 0.5)
    + center_value;
}

double TuningSignalGenerator::get_sine_signal(double elapsed_time, double amplitude,
                                              double frequency, double center_value)
{
  return -cos(elapsed_time * frequency * 2 * M_PI) * amplitude + center_value;
}

void TuningSignalGenerator::update_params()
{
  // controller_output
  std::string controller_output_string = this->get_parameter("controller_output").as_string();
  if (controller_output_string == "roll") {
    controller_output_ = ControllerOutput::ROLL;
  } else if (controller_output_string == "pitch") {
    controller_output_ = ControllerOutput::PITCH;
  } else if (controller_output_string == "altitude") {
    controller_output_ = ControllerOutput::ALTITUDE;
  } else if (controller_output_string == "course") {
    controller_output_ = ControllerOutput::COURSE;
  } else if (controller_output_string == "airspeed") {
    controller_output_ = ControllerOutput::AIRSPEED;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Param controller_output set to invalid type %s!",
                 controller_output_string.c_str());
  }

  // signal_type
  std::string signal_type_string = this->get_parameter("signal_type").as_string();
  if (signal_type_string == "step") {
    signal_type_ = SignalType::STEP;
  } else if (signal_type_string == "square") {
    signal_type_ = SignalType::SQUARE;
  } else if (signal_type_string == "sawtooth") {
    signal_type_ = SignalType::SAWTOOTH;
  } else if (signal_type_string == "triangle") {
    signal_type_ = SignalType::TRIANGLE;
  } else if (signal_type_string == "sine") {
    signal_type_ = SignalType::SINE;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Param signal_type set to invalid type %s!",
                 signal_type_string.c_str());
  }

  // publish_rate_hz
  double publish_rate_hz_value = this->get_parameter("publish_rate_hz").as_double();
  if (publish_rate_hz_value <= 0) {
    RCLCPP_ERROR(this->get_logger(), "Param publish_rate_hz must be greater than 0!");
  } else {
    // Parameter has changed, create new timer with updated value
    if (publish_rate_hz_ != publish_rate_hz_value) {
      publish_rate_hz_ = publish_rate_hz_value;
      publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<long>(1000 / publish_rate_hz_)),
        std::bind(&TuningSignalGenerator::publish_timer_callback, this));
    }
  }

  // signal_magnitude_
  signal_magnitude_ = this->get_parameter("signal_magnitude").as_double();

  // frequency_hz
  double frequency_hz_value = this->get_parameter("frequency_hz").as_double();
  if (frequency_hz_value <= 0) {
    RCLCPP_ERROR(this->get_logger(), "Param frequency_hz must be greater than 0!");
  } else {
    frequency_hz_ = frequency_hz_value;
  }

  // default_va_c
  default_va_c_ = this->get_parameter("default_va_c").as_double();

  // default_h_c
  default_h_c_ = this->get_parameter("default_h_c").as_double();

  // default_chi_c
  default_chi_c_ = this->get_parameter("default_chi_c").as_double();

  // default_theta_c
  default_theta_c_ = this->get_parameter("default_theta_c").as_double();

  // default_phi_c
  default_phi_c_ = this->get_parameter("default_phi_c").as_double();
}

void TuningSignalGenerator::reset()
{
  initial_time_ = this->get_clock()->now().seconds();
  paused_time_ = 0;
  is_paused_ = true;
  single_period_start_time_ = 0;
  step_toggled_ = false;
}

} // namespace rosplane

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rosplane::TuningSignalGenerator>());
  rclcpp::shutdown();
  return 0;
}
