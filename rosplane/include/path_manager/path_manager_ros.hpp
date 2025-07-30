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
 * @file path_manager_ros.hpp
 *
 * Base class definition for autopilot path follower in chapter 10 of UAVbook, see http://uavbook.byu.edu/doku.php
 * Implements ROS interfaces for path management 
 *
 * Version 1:
 * @author Gary Ellingson <gary.ellingson@byu.edu>
 * adapted by Judd Mehr and Brian Russel for ROSplane software
 *
 * Version 2:
 * @author Ian Reid <ian.young.reid@gmail.com>
 */

#ifndef PATH_MANAGER_ROS_H
#define PATH_MANAGER_ROS_H

#include <rclcpp/rclcpp.hpp>
#include <limits>

#include "param_manager.hpp"
#include "rosplane_msgs/msg/current_path.hpp"
#include "rosplane_msgs/msg/state.hpp"
#include "rosplane_msgs/msg/waypoint.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace rosplane
{
class PathManagerROS : public rclcpp::Node
{
public:
  PathManagerROS();

protected:
  struct Waypoint
  {
    float w[3];
    float chi_d;
    bool use_chi;
    float va_d;
  };

  std::vector<Waypoint> waypoints_; /** Vector of waypoints maintained by path_manager */
  int num_waypoints_;
  int idx_a_; /** index to the waypoint that was most recently achieved */

  bool temp_waypoint_ = false;
  int orbit_dir_ = 0;

  struct Input
  {
    float pn;  /** position north */
    float pe;  /** position east */
    float h;   /** altitude */
    float chi; /** course angle */
  };

  struct Output
  {
    bool flag;    /** Inicates strait line or orbital path (true is line, false is orbit) */
    float va_d;   /** Desired airspeed (m/s) */
    float r[3];   /** Vector to origin of straight line path (m) */
    float q[3];   /** Unit vector, desired direction of travel for line path */
    float c[3];   /** Center of orbital path (m) */
    float rho;    /** Radius of orbital path (m) */
    int8_t lamda; /** Direction of orbital path (cw is 1, ccw is -1) */
  };

  ParamManager params_; /** Holds the parameters for the path_manager and children */

  /**
   * @brief Manages the current path based on the stored waypoint list
   * 
   * @param input: Input object that contains information about the waypoint
   * @param output: Output object that contains the parameters for the desired type of line, based on the current and next waypoints
   */
  virtual void manage(const Input & input, Output & output) = 0;

private:
  rclcpp::Subscription<rosplane_msgs::msg::State>::SharedPtr
    vehicle_state_sub_; /**< vehicle state subscription */
  rclcpp::Subscription<rosplane_msgs::msg::Waypoint>::SharedPtr
    new_waypoint_sub_; /**< new waypoint subscription */
  rclcpp::Publisher<rosplane_msgs::msg::CurrentPath>::SharedPtr
    current_path_pub_; /**< controller commands publication */

  rosplane_msgs::msg::State vehicle_state_; /**< vehicle state */

  bool params_initialized_;
  bool state_init_;
  std::chrono::microseconds timer_period_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  void vehicle_state_callback(const rosplane_msgs::msg::State &
                                msg); /** subscribes to the estimated state from the estimator */
  void new_waypoint_callback(const rosplane_msgs::msg::Waypoint &
                               msg); /** subscribes to waypoint messages from the path_planner */
  void current_path_publish();       /** Publishes the current path to the path follower */

  /**
   * @brief Callback that gets triggered when a ROS2 parameter is changed
   * 
   * @param parameters: Vector of rclcpp::Parameter objects
   * 
   * @return SetParametersResult object with the success of the parameter change
   */
  rcl_interfaces::msg::SetParametersResult
  parametersCallback(const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Declares parameters with ROS2 and adds it to the parameter manager object
   */
  void declare_parameters();

  /**
   * @brief Sets up the timer with the period specified by the parameters
   */
  void set_timer();
};
} // namespace rosplane
#endif // PATH_MANAGER_ROS_H
