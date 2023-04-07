//#include <ros/ros.h>
//#include <rosplane_msgs/Waypoint.h>
#include "rclcpp/rclcpp.hpp"
#include "rosplane_msgs/msg/Waypoint.h"
#define num_waypoints 3

int main(int argc, char **argv)
{
  //ros::init(argc, argv, "rosplane_simple_path_planner");
  //ros::NodeHandle nh_; // !!! n vs nh_ example I am following used nh_ 
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rosplane_simple_path_planner")

  //ros::Publisher waypointPublisher = nh_.advertise<rosplane_msgs::Waypoint>("waypoint_path", 10);
  //ros::Rate loop_rate(10);
  //rclpp::Rate loop_rate(10); !!! probably not important but the example had this Also there is a quality of service that is recomended probably not needed. 
  auto waypointPublisher = node->create_publisher<rosplane_msgs::msg::Waypoint>("waypoint_path", 10); // !!! n vs nh_ example I am following used nh_ 

  float Va = 12;
  float wps[5*num_waypoints] =
  {
    200, 0, -50, 45*M_PI/180, Va,
    0, 200, -50, 45*M_PI/180, Va,
    200, 200, -50, 225*M_PI/180, Va,
  };

  for (int i(0); i < num_waypoints; i++)
  {
    rclpp::Duration(0.5).sleep(); // !!! pattern following ros to rclpp

    rosplane_msgs::msg::Waypoint new_waypoint; // !!! pattern following add ::msg directory

    new_waypoint.w[0] = wps[i*5 + 0];
    new_waypoint.w[1] = wps[i*5 + 1];
    new_waypoint.w[2] = wps[i*5 + 2];
    new_waypoint.chi_d = wps[i*5 + 3];

    new_waypoint.chi_valid = true;
    new_waypoint.Va_d = wps[i*5 + 4];
    if (i == 0)
      new_waypoint.set_current = true;
    else
      new_waypoint.set_current = false;
    new_waypoint.clear_wp_list = false;

    waypointPublisher.publish(new_waypoint);
  }
  rclpp::Duration(1.5).sleep(); // !!! pattern following ros to rclpp

  return 0;
}
