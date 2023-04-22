/*
 * Copyright (C) 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <ignition/msgs/entity.pb.h>
#include <ignition/msgs/wrench.pb.h>
#include <ignition/msgs/entity_wrench.pb.h>
#include <ignition/transport.hh>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/transport/Node.hh>

//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void cb(const ignition::msgs::Wrench &wrench_)
{

  ignition::transport::Node publisher;

  auto pub = publisher.Advertise<ignition::msgs::EntityWrench>("/world/rosplane2/wrench");
  auto const force = wrench_.force();
  auto const torque = wrench_.torque();

  ignition::msgs::EntityWrench ew;
  ew.mutable_entity()->set_name("fixedwing");
  ew.mutable_entity()->set_type(ignition::msgs::Entity::MODEL);
  ew.mutable_wrench()->mutable_force()->set_x(force.x());
  ew.mutable_wrench()->mutable_force()->set_y(force.y());
  ew.mutable_wrench()->mutable_force()->set_z(force.z());
  ew.mutable_wrench()->mutable_torque()->set_x(torque.x());
  ew.mutable_wrench()->mutable_torque()->set_y(torque.y());
  ew.mutable_wrench()->mutable_torque()->set_z(torque.z());
  pub.Publish(ew);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Create a transport node and advertise a topic.
  ignition::transport::Node subscriber;
  const std::string topic_sub = "/forces_moments";
  
  // Subscribe to a topic by registering a callback.
  if (!subscriber.Subscribe(topic_sub, cb))
  {
    std::cerr << "Error subscribing to topic [" << topic_sub << "]" << std::endl;
    return -1;
  }

  // Zzzzzz.
  ignition::transport::waitForShutdown();

  return 0;
}
