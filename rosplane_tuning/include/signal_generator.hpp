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

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "rosplane_msgs/msg/controller_commands.hpp"

namespace rosplane
{
/**
 * This class is used to generate various input signals to test and tune all the control layers
 * in ROSplane. It currently supports square, sawtooth, triangle, and sine signals, and supports
 * outputting to the roll, pitch, altitude, course, and airspeed controllers.
 */
class TuningSignalGenerator : public rclcpp::Node
{
public:
  /// Contructor for signal generator.
  TuningSignalGenerator();

private:
  /// This defines what controller to publish the generated signal to.
  enum class ControllerOutput
  {
    ROLL,
    PITCH,
    ALTITUDE,
    COURSE,
    AIRSPEED
  };

  /// This defines what type of signal to publish to the selected controller.
  enum class SignalType
  {
    STEP,
    SQUARE,
    SAWTOOTH,
    TRIANGLE,
    SINE
  };

  // Parameters
  ControllerOutput controller_output_; ///< Controller to output command signals to.
  SignalType signal_type_;             ///< Signal type to output.
  double publish_rate_hz_;             ///< Frequency to publish commands.
  double signal_magnitude_;            ///< The the magnitude of the signal being generated.
  double frequency_hz_;                ///< Frequency of the signal.
  double default_va_c_;                ///< Default for va_c.
  double default_h_c_;                 ///< Default for h_c.
  double default_chi_c_;               ///< Default for chi_c.
  double default_theta_c_;             ///< Default for theta_c.
  double default_phi_c_;               ///< Default for phi_c.

  // Internal values
  bool step_toggled_;               ///< Flag for when step signal has been toggled.
  double initial_time_;             ///< Initial time of the signal.
  bool is_paused_;                  ///< Flag to specify if signal should be paused.
  double paused_time_;              ///< Amount of time that has been spent paused.
  double single_period_start_time_; ///< Epoch time of when single period start was called.

  /// Controller command ROS message publisher.
  rclcpp::Publisher<rosplane_msgs::msg::ControllerCommands>::SharedPtr command_publisher_;

  /// ROS timer to run timer callback, which publishes commands
  rclcpp::TimerBase::SharedPtr publish_timer_;

  /// ROS parameter change callback handler.
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  /// ROS service for toggling step signal.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr step_toggle_service_;
  /// ROS service for reset signal.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  /// ROS service for pause signal.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_;
  /// ROS service for start signal continuously.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_continuous_service_;
  /// ROS service for start signal for one period.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_single_service_;

  /// Callback to publish command on topic.
  void publish_timer_callback();

  /// Callback for parameter changes.
  rcl_interfaces::msg::SetParametersResult
  param_callback(const std::vector<rclcpp::Parameter> & params);

  /// Callback to toggle step signal.
  bool step_toggle_service_callback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                                    const std_srvs::srv::Trigger::Response::SharedPtr & res);
  /// Callback to reset signal.
  bool reset_service_callback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                              const std_srvs::srv::Trigger::Response::SharedPtr & res);
  /// Callback to pause signal.
  bool pause_service_callback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                              const std_srvs::srv::Trigger::Response::SharedPtr & res);
  /// Callback to start signal continuously.
  bool start_continuous_service_callback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                                         const std_srvs::srv::Trigger::Response::SharedPtr & res);
  /// Callback to start signal for a single period.
  bool start_single_service_callback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                                     const std_srvs::srv::Trigger::Response::SharedPtr & res);

  /**
   * @brief Get the value for a step signal at the given time with the given conditions.
   *
   * @param step_toggled Flag to specify if signal is "stepped up" or not.
   * @param amplitude The amplitude of the signal.
   * @param center_value The central value of the signal. Not the initial value of the signal,
   *  but the value directly in the middle of the step values.
   */
  static double get_step_signal(bool step_toggled, double amplitude, double center_value);

  /**
   * @brief Get the value for a square signal at the given time with the given conditions.
   *
   * @param elapsed_time The amount of time that has passed since the 'start' of the signal
   *   in seconds.
   * @param amplitude The amplitude of the signal.
   * @param frequency The frequency of the signal.
   * @param center_value The central value of the signal. The in other words, the signal 'offset'.
   */
  static double get_square_signal(double elapsed_time, double amplitude, double frequency,
                                  double center_value);
  /**
   * @brief Get the value for a sawtooth signal at the given time with the given conditions.
   *
   * @param elapsed_time The amount of time that has passed since the 'start' of the signal
   *   in seconds.
   * @param amplitude The amplitude of the signal.
   * @param frequency The frequency of the signal.
   * @param center_value The central value of the signal. The in other words, the signal 'offset'.
   */
  static double get_sawtooth_signal(double elapsed_time, double amplitude, double frequency,
                                    double center_value);
  /**
   * @brief Get the value for a triangle signal at the given time with the given conditions.
   *
   * @param elapsed_time The amount of time that has passed since the 'start' of the signal
   *   in seconds.
   * @param amplitude The amplitude of the signal.
   * @param frequency The frequency of the signal.
   * @param center_value The central value of the signal. The in other words, the signal 'offset'.
   */
  static double get_triangle_signal(double elapsed_time, double amplitude, double frequency,
                                    double center_value);
  /**
   * @brief Get the value for a sine signal at the given time with the given conditions.
   *
   * @param elapsed_time The amount of time that has passed since the 'start' of the signal
   *   in seconds.
   * @param amplitude The amplitude of the signal.
   * @param frequency The frequency of the signal.
   * @param center_value The central value of the signal. The in other words, the signal 'offset'.
   */
  static double get_sine_signal(double elapsed_time, double amplitude, double frequency,
                                double center_value);

  /// Updates the parameters within the class with the latest values from ROS.
  void update_params();

  /// Reset the signal generator.
  void reset();
};
} // namespace rosplane

#endif // TUNING_SIGNAL_GENERATOR_HPP
