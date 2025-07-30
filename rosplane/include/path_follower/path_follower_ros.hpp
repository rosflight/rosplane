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
 * @file path_follower_lines_orbits.hpp
 * 
 * Implements ROS interfaces for path following.
 * 
 * @author Ian Reid <ian.young.reid@gmail.com>
*/

#ifndef PATH_FOLLOWER_ROS_H
#define PATH_FOLLOWER_ROS_H

#include <rclcpp/rclcpp.hpp>

#include "param_manager.hpp"
#include "rosplane_msgs/msg/controller_commands.hpp"
#include "rosplane_msgs/msg/current_path.hpp"
#include "rosplane_msgs/msg/state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace rosplane
{

/**
 * This defines if the given path is an orbit or a line.
 */
enum class PathType
{
  ORBIT,
  LINE
};

/**
 * This class implements all of the ROS2 interfaces for path following.
 */
class PathFollowerROS : public rclcpp::Node
{
public:
  /**
   * The constructor of node.
   */
  PathFollowerROS();

protected:
  /**
   * Defines the path that is currently being followed.
   */
  struct Input
  {
    PathType p_type;  /**< Type of path (indicates if orbit or straight line related values are valid). */
    float va_d; /**< Desired airspeed along this section of path. */
    float r_path[3]; /**< The position of the tail of q (where the straight path starts). */
    float q_path[3]; /**< Unit vector direction of the straight path. */
    float c_orbit[3]; /**< Center of the orbit in NED. */
    float rho_orbit; /**< Radius of the orbit. */
    int lam_orbit; /**< The direction the orbit will go CW (1) or CCW (-1). */
    float pn;  /**< Position north  of the aircraft*/
    float pe;  /**< Position east  of the aircraft*/
    float h;   /**< Altitude  of the aircraft*/
    float va;  /**< Airspeed of the aircraft*/
    float chi; /**< Course angle of the aircraft (rad)*/
    float psi; /**< Heading angle of the aircraft (rad)*/
  };

  /**
   * The controller commands needed to acheive the input.
   */
  struct Output
  {
    double va_c;   /** Commanded airspeed (m/s) */
    double h_c;    /** Commanded altitude (m) */
    double chi_c;  /** Commanded course (rad) */
    double phi_ff; /** Feed forward term for orbits (rad) */
  };

  /**
   * The path following algorithm the child will implement.
   */
  virtual void follow(const Input & input, Output & output) = 0;

  /**
   * The param manager for the ROS parameters.
   */
  ParamManager params_;

private:
  /**
   * Subscribes to state from the estimator
   */
  rclcpp::Subscription<rosplane_msgs::msg::State>::SharedPtr vehicle_state_sub_;

  /** 
   * Subscribes to the current_path topic from the path manager
   */
  rclcpp::Subscription<rosplane_msgs::msg::CurrentPath>::SharedPtr current_path_sub_;

  /**
   * Publishes commands to the controller
   */
  rclcpp::Publisher<rosplane_msgs::msg::ControllerCommands>::SharedPtr controller_commands_pub_;

  /**
   * The period of the timer in microseconds.
   */
  std::chrono::microseconds timer_period_;
  
  /**
   * The timer that indicates how often the commands should be updated.
   */
  rclcpp::TimerBase::SharedPtr update_timer_;

  /**
   * Indicates if the params have been initialized.
   */
  bool params_initialized_;

  /**
   * Indicates if the state has been initialized.
   */
  bool state_init_;
  
  /**
   * Indicates if the current path has been initialized.
   */
  bool current_path_init_;

  /**
   * Handle for the function that propagates changes in parameters to the the appropriate 
   * member variables and functions.
   */
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  
  /**
   * The controller commands that will be sent to the controller.
   */
  rosplane_msgs::msg::ControllerCommands controller_commands_;
  
  /**
   * Current input to the follower.
   */
  Input input_;

  /**
   * @brief Sets the timer with the timer period as specified by the ROS2 parameters
   */
  void set_timer();

  /**
   * @brief Callback for the subscribed state messages from the estimator
   */
  void vehicle_state_callback(const rosplane_msgs::msg::State::SharedPtr msg);

  /**
   * @brief Callback for the subscribed current_path messages from the path_manager
   */
  void current_path_callback(const rosplane_msgs::msg::CurrentPath::SharedPtr msg);

  /**
   * @brief Calculates and publishes the commands messages
   */
  void update();

  /**
   * @brief Callback for when ROS2 parameters change.
   * 
   * @param Vector of rclcpp::Parameter objects that have changed
   * @return SetParametersResult object that describes the success or failure of the request
   */
  rcl_interfaces::msg::SetParametersResult
  parametersCallback(const std::vector<rclcpp::Parameter> & parameters);

  /**
   * This declares each parameter as a parameter so that the ROS2 parameter system can recognize each parameter.
   * It also sets the default parameter, which can be overridden by a parameter file
   */
  void declare_parameters();
};

} // namespace rosplane

#endif // PATH_FOLLOWER_ROS_H
