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
 * ROS2 node for generating various input signals for tuning any/all layers of the controller.
 *
 * @author Brandon Sutherland <brandonsutherland2@gmail.com>
 */

#ifndef TUNING_SIGNAL_GENERATOR_HPP
#define TUNING_SIGNAL_GENERATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include "rosplane_msgs/msg/controller_commands.hpp"
#include "rosplane_msgs/msg/controller_internals_debug.hpp"

namespace rosplane
{
/**
 * This class is used to generate various input signals to test and tune all the control layers
 * in ROSplane. It currently supports square, sawtooth, triangle, and sine signals, and supports
 * outputting to the roll, pitch, altitude, heading, and airspeed controllers. 
 */
class TuningSignalGenerator : public rclcpp::Node
{
public: 
  /**
   * @brief Contructor for signal generator.
   */
  TuningSignalGenerator();

private:
  /**
   * This defines what controller to publish the generated signal to.
   */
  enum class ControllerOutput {
    ROLL,
    PITCH,
    ALTITUDE,
    HEADING,
    AIRSPEED
  };
  
  /**
   * This defines what type of signal to publish to the selected controller.
   */
  enum class SignalType {
    SQUARE,
    SAWTOOTH,
    TRIANGLE,
    SINE
  };

  ControllerOutput controller_output_;  ///< Controller to output command signals to.
  SignalType signal_type_;              ///< Signal type to output.
  double dt_hz_;                        ///< Frequency to publish commands.
  double amplitude_;                    ///< Amplitude of signal.
  double frequency_hz_;                 ///< Frequency of the signal.
  double offset_;                       ///< Offset of signal from 0.
  double initial_time_;                 ///< Initial time of the signal.

  /// Controller command ROS message publisher.
  rclcpp::Publisher<rosplane_msgs::msg::ControllerCommands>::SharedPtr command_publisher_;
  /// Controller internals ROS message publisher.
  rclcpp::Publisher<rosplane_msgs::msg::ControllerInternalsDebug>::SharedPtr internals_publisher_;
  
  /// ROS timer to run timer callback, which publishes commands
  rclcpp::TimerBase::SharedPtr publish_timer_;

  /**
   * Callback to publish command on topic.
   */
  void publish_timer_callback();

  /**
   * @brief Get the value for a square signal at the given time with the given conditions.
   *
   * @param elapsed_time The amount of time that has passed since the 'start' of the signal
   *   in seconds.
   * @param amplitude The amplitude of the signal.
   * @param frequency The frequency of the signal.
   * @param initial_value Inital value of the signal. The in other words, the signal 'offset'.
   */
  static double get_square_signal(double elapsed_time, double amplitude, double frequency, 
                                  double initial_value);
  /**
   * @brief Get the value for a sawtooth signal at the given time with the given conditions.
   *
   * @param elapsed_time The amount of time that has passed since the 'start' of the signal
   *   in seconds.
   * @param amplitude The amplitude of the signal.
   * @param frequency The frequency of the signal.
   * @param initial_value Inital value of the signal. The in other words, the signal 'offset'.
   */
  static double get_sawtooth_signal(double elapsed_time, double amplitude, double frequency,
                                    double initial_value);
  /**
   * @brief Get the value for a triangle signal at the given time with the given conditions.
   *
   * @param elapsed_time The amount of time that has passed since the 'start' of the signal
   *   in seconds.
   * @param amplitude The amplitude of the signal.
   * @param frequency The frequency of the signal.
   * @param initial_value Inital value of the signal. The in other words, the signal 'offset'.
   */
  static double get_triangle_signal(double elapsed_time, double amplitude, double frequency,
                                    double initial_value);
  /**
   * @brief Get the value for a sine signal at the given time with the given conditions.
   *
   * @param elapsed_time The amount of time that has passed since the 'start' of the signal
   *   in seconds.
   * @param amplitude The amplitude of the signal.
   * @param frequency The frequency of the signal.
   * @param initial_value Inital value of the signal. The in other words, the signal 'offset'.
   */
  static double get_sine_signal(double elapsed_time, double amplitude, double frequency,
                                double initial_value);

  /**
   * Updates the parameters within the class with the latest values from ROS.
   */
  void update_params();
};
} // namespace rosplane

#endif // TUNING_SIGNAL_GENERATOR_HPP
