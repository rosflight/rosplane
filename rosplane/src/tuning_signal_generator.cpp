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
#include <string>
#include <cmath>

#include "tuning_signal_generator.hpp"

namespace rosplane
{
TuningSignalGenerator::TuningSignalGenerator() :
    Node("signal_generator"),
    controller_output_(ControllerOutput::ROLL),
    signal_type_(SignalType::SQUARE),
    dt_hz_(0),
    amplitude_(0),
    frequency_hz_(0),
    offset_(0),
    initial_time_(0)
{
  this->declare_parameter("controller_output", "roll");
  this->declare_parameter("signal_type", "square");
  this->declare_parameter("dt_hz", 100.0);
  this->declare_parameter("amplitude", 1.0);
  this->declare_parameter("frequency_hz", 0.2);
  this->declare_parameter("offset", 0.0);

  update_params();
  initial_time_ = this->get_clock()->now().seconds();

  publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<long>(1000 / dt_hz_)), 
      std::bind(&TuningSignalGenerator::publish_timer_callback, this));

  reset_service_ = this->create_service<std_srvs::srv::Trigger>("reset_signal", 
      std::bind(&TuningSignalGenerator::reset_service_callback, this, 
                std::placeholders::_1, std::placeholders::_2));
  pause_service_ = this->create_service<std_srvs::srv::Trigger>("pause_signal",
      std::bind(&TuningSignalGenerator::pause_service_callback, this, 
                std::placeholders::_1, std::placeholders::_2));
  start_continuous_service_ = this->create_service<std_srvs::srv::Trigger>("start_continuous_signal",
      std::bind(&TuningSignalGenerator::start_continuous_service_callback, this,
                std::placeholders::_1, std::placeholders::_2));
  start_single_service_ = this->create_service<std_srvs::srv::Trigger>("start_single_period",
      std::bind(&TuningSignalGenerator::start_single_service_callback, this,
                std::placeholders::_1, std::placeholders::_2));
}


void TuningSignalGenerator::publish_timer_callback() {
  update_params();

  double signal_value = 0;
  double elapsed_time = this->get_clock()->now().seconds() - initial_time_;

  // Get value for signal
  switch (signal_type_) {
    case SignalType::SQUARE:
      signal_value = get_square_signal(elapsed_time, amplitude_, frequency_hz_, offset_);
      break;
    case SignalType::SAWTOOTH:
      signal_value = get_sawtooth_signal(elapsed_time, amplitude_, frequency_hz_, offset_);
      break;
    case SignalType::TRIANGLE:
      signal_value = get_triangle_signal(elapsed_time, amplitude_, frequency_hz_, offset_);
      break;
    case SignalType::SINE:
      signal_value = get_sine_signal(elapsed_time, amplitude_, frequency_hz_, offset_);
      break;
  }
  
  // Create required publishers if they don't already exist
  if (controller_output_ == ControllerOutput::ROLL || 
      controller_output_ == ControllerOutput::PITCH) {
    if (internals_publisher_ == nullptr) {
      internals_publisher_ = 
          this->create_publisher<rosplane_msgs::msg::ControllerInternalsDebug>("/tuning_debug", 1);
    }
  } else {
    if (command_publisher_ == nullptr) {
      command_publisher_ = 
          this->create_publisher<rosplane_msgs::msg::ControllerCommands>("/controller_command", 1);
    }
  }

  // Publish message
  switch (controller_output_) {
    case ControllerOutput::ROLL: {
        rosplane_msgs::msg::ControllerInternalsDebug message; 
        message.header.stamp = this->get_clock()->now();
        message.phi_c = signal_value;
        internals_publisher_->publish(message);
        break;
      }
    case ControllerOutput::PITCH: {
        rosplane_msgs::msg::ControllerInternalsDebug message; 
        message.header.stamp = this->get_clock()->now();
        message.theta_c = signal_value;
        internals_publisher_->publish(message);
        break;
      }
    case ControllerOutput::ALTITUDE: {
        rosplane_msgs::msg::ControllerCommands message; 
        message.header.stamp = this->get_clock()->now();
        message.h_c = signal_value;
        command_publisher_->publish(message);
        break;
      }
    case ControllerOutput::HEADING: {
        rosplane_msgs::msg::ControllerCommands message; 
        message.header.stamp = this->get_clock()->now();
        message.chi_c = signal_value;
        command_publisher_->publish(message);
        break;
      }
    case ControllerOutput::AIRSPEED: {
        rosplane_msgs::msg::ControllerCommands message; 
        message.header.stamp = this->get_clock()->now();
        message.va_c = signal_value;
        command_publisher_->publish(message);
        break;
      }
  }
}


bool TuningSignalGenerator::reset_service_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr & req,
    const std_srvs::srv::Trigger::Response::SharedPtr & res) {

  res->success = true;
  return true;
}


bool TuningSignalGenerator::pause_service_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr & req,
    const std_srvs::srv::Trigger::Response::SharedPtr & res) {

  res->success = true;
  return true;
}


bool TuningSignalGenerator::start_continuous_service_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr & req,
    const std_srvs::srv::Trigger::Response::SharedPtr & res) {

  res->success = true;
  return true;
}


bool TuningSignalGenerator::start_single_service_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr & req,
    const std_srvs::srv::Trigger::Response::SharedPtr & res) {

  res->success = true;
  return true;
}


double TuningSignalGenerator::get_square_signal(double elapsed_time, double amplitude, 
                                                double frequency, double offset) {
  // amplitude * (1 to -1 switching value) + offset
  return amplitude * ((static_cast<int>(elapsed_time * frequency * 2) % 2) * 2 - 1) + offset;
}


double TuningSignalGenerator::get_sawtooth_signal(double elapsed_time, double amplitude, 
                                                  double frequency, double offset) {
  // slope * elapsed_time - num_cycles * offset_per_cycle + offset
  return 2 * amplitude * (elapsed_time * frequency - static_cast<int>(elapsed_time * frequency) - 
      0.5) + offset;
}


double TuningSignalGenerator::get_triangle_signal(double elapsed_time, double amplitude,
                                                  double frequency, double offset) {
  // (1 to -1 switching value) * sawtooth_at_twice_the_rate + offset
  return ((static_cast<int>(elapsed_time * frequency * 2) % 2) * 2 - 1) * 2 * 
      amplitude * (2 * elapsed_time * frequency - static_cast<int>(2 * elapsed_time * frequency) - 
      0.5) + offset;
}


double TuningSignalGenerator::get_sine_signal(double elapsed_time, double amplitude, 
                                              double frequency, double offset) {
  return sin(elapsed_time * frequency * 2 * M_PI) * amplitude + offset;
}


void TuningSignalGenerator::update_params() {
  // controller output
  std::string controller_output_string = 
      this->get_parameter("controller_output").as_string();
  if (controller_output_string == "roll") {
    controller_output_ = ControllerOutput::ROLL;
  } else if (controller_output_string == "pitch") {
    controller_output_ = ControllerOutput::PITCH;
  } else if (controller_output_string == "altitude") {
    controller_output_ = ControllerOutput::ALTITUDE;
  } else if (controller_output_string == "heading") {
    controller_output_ = ControllerOutput::HEADING;
  } else if (controller_output_string == "airspeed") {
    controller_output_ = ControllerOutput::AIRSPEED;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Param controller_output set to invalid type %s!", 
                 controller_output_string.c_str());
  }

  // signal type
  std::string signal_type_string = this->get_parameter("signal_type").as_string();
  if (signal_type_string == "square") {
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

  // dt hz
  double dt_hz_value = this->get_parameter("dt_hz").as_double();
  if (dt_hz_value <= 0) {
    RCLCPP_ERROR(this->get_logger(), "Param dt_hz must be greater than 0!");
  } else {
    // Parameter has changed, create new timer with updated value
    if (dt_hz_ != dt_hz_value) {
      dt_hz_ = dt_hz_value;
      publish_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(static_cast<long>(1000 / dt_hz_)), 
          std::bind(&TuningSignalGenerator::publish_timer_callback, this));
    }
  }
  
  // amplitude
  amplitude_ = this->get_parameter("amplitude").as_double();
  
  // frequency hz
  double frequency_hz_value = this->get_parameter("frequency_hz").as_double();
  if (frequency_hz_value <= 0) {
    RCLCPP_ERROR(this->get_logger(), "Param frequency_hz must be greater than 0!");
  } else {
    frequency_hz_ = frequency_hz_value;
  }
  
  // offset
  offset_ = this->get_parameter("offset").as_double();
}
} // namespace rosplane


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rosplane::TuningSignalGenerator>());
  rclcpp::shutdown();
  return 0;
}
