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

    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<long>(1000 / dt_hz_)), 
        std::bind(&TuningSignalGenerator::publish_timer_callback, this));
  }

void TuningSignalGenerator::publish_timer_callback() {

}

double TuningSignalGenerator::get_square_signal(double elapsed_time, double amplitude, 
                                                double frequency, double initial_value) {

}

double TuningSignalGenerator::get_sawtooth_signal(double elapsed_time, double amplitude, 
                                                  double frequency, double initial_value) {

}

double TuningSignalGenerator::get_triangle_signal(double elapsed_time, double amplitude,
                                                  double frequency, double initial_value) {

}

double TuningSignalGenerator::get_sine_signal(double elapsed_time, double amplitude, 
                                              double frequency, double initial_value) {

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
                 controller_output_string);
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
                 signal_type_string);
  }

  // dt hz
  dt_hz_ = this->get_parameter("dt_hz").as_double();
  
  // amplitude
  amplitude_ = this->get_parameter("amplitude").as_double();
  
  // frequency hz
  frequency_hz_ = this->get_parameter("frequency_hz").as_double();
  
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
