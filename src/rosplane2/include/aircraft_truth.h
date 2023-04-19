/*
 * Copyright 2016 Gary Ellingson, Brigham Young University, Provo UT
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef ROSPLANE2_AIRCRAFT_TRUTH_H
#define ROSPLANE2_AIRCRAFT_TRUTH_H

// ******** Old ROS1 Includes********
// #include <stdio.h>

// #include <boost/bind.hpp>
// #include <eigen3/Eigen/Eigen>

// #include <gazebo/common/common.hh>
// #include <gazebo/common/Plugin.hh>
// #include <gazebo/gazebo.hh>
// #include <gazebo/physics/physics.hh>
// #include <ros/callback_queue.h>
// #include <ros/ros.h>

// #include <rosplane2_msgs/State.h>
// #include <std_msgs/Float32.h>
// #include <geometry_msgs/Vector3.h>

// #include "rosplane_sim/common.h"
// #include "rosplane_sim/gz_compat.h"

// ****** New Includes ********
// from tutorial: standard C++ headers?
#include <chrono>
#include <functional>
#include <memory>
#include <string> 

#include "rclcpp/rclcpp.hpp" // from tutorial: "allows you to use the most common pieces of the ROS 2 system"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace rosplane2
{

class AircraftTruth : public rclcpp::Node
{
public:
  AircraftTruth();

protected:
  void PublishTruth();
  void OnUpdate(const common::UpdateInfo & /*_info*/); // need o fix the input to this

private:
  struct Wind
  {
    double N;
    double E;
    double D;
  } wind_;

  double sampling_time_;
  double prev_sim_time_;

  rclcpp::Subscriber<geometry_msgs::Vector3>::SharedPtr wind_speed_sub_; // probably wrong
  rclcpp::Publisher<rosplane_msgs::State>::SharedPtr true_state_pub_;

  void WindSpeedCallback(const geometry_msgs::Vector3 &wind);

  void topic_callback(const nav_msgs::msg::Odometry & topic_msg)

};
}

#endif // ROSPLANE_SIM_AIRCRAFT_TRUTH_H
