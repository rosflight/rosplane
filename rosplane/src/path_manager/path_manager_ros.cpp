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
 * @file path_manager_ros.cpp
 *
 * Base class definition for autopilot path follower in chapter 10 of UAVbook, see http://uavbook.byu.edu/doku.php
 * implements the ROS interfaces for path management
 *
 * @author Gary Ellingson <gary.ellingson@byu.edu>
 * adapted by Judd Mehr and Brian Russel for ROSplane software
 * @author Ian Reid <ian.young.reid@gmail.com>
 */

#include "path_manager/path_manager_dubins_fillets.hpp"

#include "path_manager/path_manager_ros.hpp"

namespace rosplane
{

PathManagerROS::PathManagerROS()
    : Node("path_manager")
    , params_(this)
    , params_initialized_(false)
{
  vehicle_state_sub_ = this->create_subscription<rosplane_msgs::msg::State>(
    "estimated_state", 10, std::bind(&PathManagerROS::vehicle_state_callback, this, _1));
  new_waypoint_sub_ = this->create_subscription<rosplane_msgs::msg::Waypoint>(
    "waypoint_path", 10, std::bind(&PathManagerROS::new_waypoint_callback, this, _1));
  current_path_pub_ = this->create_publisher<rosplane_msgs::msg::CurrentPath>("current_path", 10);

  // Set the parameter callback, for when parameters are changed.
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&PathManagerROS::parametersCallback, this, std::placeholders::_1));

  // Declare parameters maintained by this node with ROS2. Required for all ROS2 parameters associated with this node
  declare_parameters();
  params_.set_parameters();

  params_initialized_ = true;

  // Now that the update rate has been updated in parameters, create the timer
  set_timer();

  num_waypoints_ = 0;

  state_init_ = false;
}

void PathManagerROS::declare_parameters()
{
  params_.declare_double("R_min", 50.0);
  params_.declare_double("current_path_pub_frequency", 100.0);
  params_.declare_double("default_altitude", 50.0);
  params_.declare_double("default_airspeed", 15.0);
}

void PathManagerROS::set_timer()
{
  // Calculate the period in milliseconds from the frequency
  double frequency = params_.get_double("current_path_pub_frequency");
  timer_period_ = std::chrono::microseconds(static_cast<long long>(1.0 / frequency * 1e6));

  update_timer_ =
    rclcpp::create_timer(this, this->get_clock(), timer_period_, std::bind(&PathManagerROS::current_path_publish, this));
}

rcl_interfaces::msg::SetParametersResult
PathManagerROS::parametersCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "One of the parameters given is not a parameter of the controller node.";

  // Update the changed parameters in the param_manager object
  bool success = params_.set_parameters_callback(parameters);
  if (success) {
    result.successful = true;
    result.reason = "success";
  }

  // If the frequency parameter was changed, restart the timer.
  if (params_initialized_ && success) {
    double frequency = params_.get_double("current_path_pub_frequency");
    std::chrono::microseconds curr_period =
      std::chrono::microseconds(static_cast<long long>(1.0 / frequency * 1e6));
    if (timer_period_ != curr_period) {
      update_timer_->cancel();
      set_timer();
    }
  }

  return result;
}

void PathManagerROS::vehicle_state_callback(const rosplane_msgs::msg::State & msg)
{

  vehicle_state_ = msg;

  state_init_ = true;
}

void PathManagerROS::new_waypoint_callback(const rosplane_msgs::msg::Waypoint & msg)
{
  double R_min = params_.get_double("R_min");
  double default_altitude = params_.get_double("default_altitude");
  orbit_dir_ = 0;

  // If the message contains "clear_wp_list", then clear all waypoints and do nothing else
  if (msg.clear_wp_list == true) {
    waypoints_.clear();
    num_waypoints_ = 0;
    idx_a_ = 0;
    return;
  }

  // If there are currently no waypoints in the list, then add a temporary waypoint as
  // the current state of the aircraft. This is necessary to define a line for line following.
  if (waypoints_.size() == 0) {
    Waypoint temp_waypoint;

    temp_waypoint.w[0] = vehicle_state_.position[0];
    temp_waypoint.w[1] = vehicle_state_.position[1];

    if (vehicle_state_.position[2] < -default_altitude) {

      temp_waypoint.w[2] = vehicle_state_.position[2];
    } else {
      temp_waypoint.w[2] = -default_altitude;
    }

    temp_waypoint.chi_d = 0.0; // Doesn't matter, it is never used.
    temp_waypoint.use_chi = false;
    temp_waypoint.va_d = msg.va_d; // Use the va_d for the next waypoint.

    waypoints_.push_back(temp_waypoint);
    num_waypoints_++;
    temp_waypoint_ = true;
  }

  // Add a default comparison for the last waypoint for feasiblity check.
  Waypoint nextwp;
  Eigen::Vector3f w_existing(std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity());
  nextwp.w[0] = msg.w[0];
  nextwp.w[1] = msg.w[1];
  nextwp.w[2] = msg.w[2];
  nextwp.chi_d = msg.chi_d;
  nextwp.use_chi = msg.use_chi;
  nextwp.va_d = msg.va_d;

  // Save the last waypoint for comparison.
  if (waypoints_.size() > 0) {
    Waypoint waypoint = waypoints_.back();
    w_existing << waypoint.w[0], waypoint.w[1], waypoint.w[2];
  }
  waypoints_.push_back(nextwp);
  num_waypoints_++;

  // Warn if too close to the last waypoint.
  Eigen::Vector3f w_new(msg.w[0], msg.w[1], msg.w[2]);

  if ((w_new - w_existing).norm() < R_min) {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "A waypoint is too close to the next waypoint. Indices: "
                         << waypoints_.size() - 2 << ", " << waypoints_.size() - 1);
  }
}

void PathManagerROS::current_path_publish()
{

  Input input;
  input.pn = vehicle_state_.position[0]; // position north
  input.pe = vehicle_state_.position[1]; // position east
  input.h = -vehicle_state_.position[2]; // altitude
  input.chi = vehicle_state_.chi;

  Output output;
  output.va_d = 0;
  output.r[0] = 0;
  output.r[1] = 0;
  output.r[2] = 0;
  output.q[0] = 0;
  output.q[1] = 0;
  output.q[2] = 0;
  output.c[0] = 0;
  output.c[1] = 0;
  output.c[2] = 0;

  if (state_init_ == true) {
    manage(input, output);
  }

  rosplane_msgs::msg::CurrentPath current_path;

  rclcpp::Time now = this->get_clock()->now();

  // Populate current_path message
  current_path.header.stamp = now;
  if (output.flag) {
    current_path.path_type = current_path.LINE_PATH;
  } else {
    current_path.path_type = current_path.ORBIT_PATH;
  }
  current_path.va_d = output.va_d;
  for (int i = 0; i < 3; i++) {
    current_path.r[i] = output.r[i];
    current_path.q[i] = output.q[i];
    current_path.c[i] = output.c[i];
  }
  current_path.rho = output.rho;
  current_path.lamda = output.lamda;

  current_path_pub_->publish(current_path);
}

} // namespace rosplane

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rosplane::PathManagerDubinsFillets>());

  return 0;
}
