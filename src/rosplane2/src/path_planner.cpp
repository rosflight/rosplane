#include "rclcpp/rclcpp.hpp"
#include "rosplane2_msgs/msg/waypoint.hpp"

#define num_waypoints 3

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rosplane2_simple_path_planner");

  auto waypoint_publisher = node->create_publisher<rosplane2_msgs::msg::Waypoint>("waypoint_path", 10); // !!! n vs nh_ example I am following used n 
  rclcpp::Rate loop_rate(10);
  float va = 35;
  float wps[5*num_waypoints] =
  {
    200, 0, -100, 45*M_PI/180, va,
    0, 200, -100, 45*M_PI/180, va,
    200, 200, -100, 225*M_PI/180, va,
  };

  rclcpp::Clock wait;
  for (int i(0); i < num_waypoints; i++)
  {

    wait.sleep_for(rclcpp::Duration(0,int(5e8)));

    rosplane2_msgs::msg::Waypoint new_waypoint;

    rclcpp::Time now = node->get_clock()->now();

    new_waypoint.header.stamp = now;

    new_waypoint.w[0] = wps[i*5 + 0];
    new_waypoint.w[1] = wps[i*5 + 1];
    new_waypoint.w[2] = wps[i*5 + 2];
    new_waypoint.chi_d = wps[i*5 + 3];

    new_waypoint.chi_valid = true;
    new_waypoint.va_d = wps[i*5 + 4];
    if (i == 0)
      new_waypoint.set_current = true;
    else
      new_waypoint.set_current = false;
    new_waypoint.clear_wp_list = false;

    RCLCPP_DEBUG_STREAM(node->get_logger(), "Publishing Waypoint!");

    RCLCPP_DEBUG_STREAM(node->get_logger(), "North: " << new_waypoint.w[0]);
    RCLCPP_DEBUG_STREAM(node->get_logger(), "East: " << new_waypoint.w[1]);
    RCLCPP_DEBUG_STREAM(node->get_logger(), "Down: " << new_waypoint.w[2]);
    RCLCPP_DEBUG_STREAM(node->get_logger(), "chi_d: " << new_waypoint.chi_d);


    waypoint_publisher->publish(new_waypoint);
  }

  wait.sleep_for(rclcpp::Duration(1,int(5e8)));
  return 0;
}
