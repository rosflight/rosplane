#include "rclcpp/rclcpp.hpp"
#include "rosplane_msgs/msg/waypoint.hpp"
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/utilities.hpp>
#include <rosplane_msgs/msg/detail/waypoint__struct.hpp>
#include <std_srvs/srv/trigger.hpp>

#define num_waypoints 7
#define num_waypoints_to_publish 3

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace rosplane
{

class path_planner : public rclcpp::Node
{
public:
  path_planner();
  ~path_planner();

private:
  rclcpp::Publisher<rosplane_msgs::msg::Waypoint>::SharedPtr waypoint_publisher;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr waypoint_service;
  rclcpp::TimerBase::SharedPtr timer_;

  bool publish_next_waypoint(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                             const std_srvs::srv::Trigger::Response::SharedPtr & res);

  void waypoint_publish();

  int num_waypoints_published;

  float wps[5 * num_waypoints];
};

path_planner::path_planner()
    : Node("path_planner")
{

  rclcpp::QoS qos_transient_local_10_(10);
  qos_transient_local_10_.transient_local();
  waypoint_publisher = this->create_publisher<rosplane_msgs::msg::Waypoint>("waypoint_path", qos_transient_local_10_);

  waypoint_service = this->create_service<std_srvs::srv::Trigger>(
    "publish_next_waypoint",
    std::bind(&path_planner::publish_next_waypoint, this, std::placeholders::_1,
              std::placeholders::_2));

  timer_ = this->create_wall_timer(1000ms, std::bind(&path_planner::waypoint_publish, this));

  num_waypoints_published = 0;

  float va = 25;
  float wps_to_add[5 * num_waypoints] = {
    0, 100,  -90, 45 * M_PI / 180, va, 0, 300,  -90, 45 * M_PI / 180, va,
    0, 500,  -90, 45 * M_PI / 180, va, 0, 700,  -90, 45 * M_PI / 180, va,
    0, 900,  -90, 45 * M_PI / 180, va, 0, 1100, -90, 45 * M_PI / 180, va,
    0, 1300, -90, 45 * M_PI / 180, va};

  for (int i = 0; i < 5 * num_waypoints; i++) { wps[i] = wps_to_add[i]; }
}

path_planner::~path_planner() {}

bool path_planner::publish_next_waypoint(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                                         const std_srvs::srv::Trigger::Response::SharedPtr & res)
{

  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Publishing next waypoint, num_waypoints_published: " << num_waypoints_published);

  if (num_waypoints_published < num_waypoints) {
    rosplane_msgs::msg::Waypoint new_waypoint;

    rclcpp::Time now = this->get_clock()->now();

    new_waypoint.header.stamp = now;

    int i = num_waypoints_published;

    new_waypoint.w[0] = wps[i * 5 + 0];
    new_waypoint.w[1] = wps[i * 5 + 1];
    new_waypoint.w[2] = wps[i * 5 + 2];
    new_waypoint.chi_d = wps[i * 5 + 3];

    new_waypoint.chi_valid = false;
    new_waypoint.va_d = wps[i * 5 + 4];
    if (i == 0) new_waypoint.set_current = true;
    else
      new_waypoint.set_current = false;
    new_waypoint.clear_wp_list = false;

    waypoint_publisher->publish(new_waypoint);

    num_waypoints_published++;
  }

  return true;
}

void path_planner::waypoint_publish()
{

  if (num_waypoints_published < num_waypoints_to_publish) {

    rosplane_msgs::msg::Waypoint new_waypoint;

    rclcpp::Time now = this->get_clock()->now();

    new_waypoint.header.stamp = now;

    int i = num_waypoints_published;

    new_waypoint.w[0] = wps[i * 5 + 0];
    new_waypoint.w[1] = wps[i * 5 + 1];
    new_waypoint.w[2] = wps[i * 5 + 2];
    new_waypoint.chi_d = wps[i * 5 + 3];

    new_waypoint.chi_valid = false;
    new_waypoint.va_d = wps[i * 5 + 4];
    if (i == 0) new_waypoint.set_current = true;
    else
      new_waypoint.set_current = false;
    new_waypoint.clear_wp_list = false;

    waypoint_publisher->publish(new_waypoint);

    num_waypoints_published++;
  }
}
} // namespace rosplane

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rosplane::path_planner>();

  rclcpp::spin(node);

  return 0;
}
