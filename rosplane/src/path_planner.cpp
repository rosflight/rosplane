#include "rclcpp/rclcpp.hpp"
#include "rosplane_msgs/msg/waypoint.hpp"
#include "rosplane_msgs/srv/add_waypoint.hpp"
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/utilities.hpp>
#include <rosplane_msgs/msg/detail/waypoint__struct.hpp>
#include <std_srvs/srv/trigger.hpp>
// #include <vector>

#define NUM_WAYPOINTS_TO_PUBLISH_AT_START 3

using std::placeholders::_1;
using std::placeholders::_2;
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
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr next_waypoint_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_waypoint_service;
  rclcpp::Service<rosplane_msgs::srv::AddWaypoint>::SharedPtr add_waypoint_service;
  rclcpp::TimerBase::SharedPtr timer_;

  bool publish_next_waypoint(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                             const std_srvs::srv::Trigger::Response::SharedPtr & res);
  
  bool update_path(const rosplane_msgs::srv::AddWaypoint::Request::SharedPtr & req,
                   const rosplane_msgs::srv::AddWaypoint::Response::SharedPtr & res);
  
  bool clear_path(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                  const std_srvs::srv::Trigger::Response::SharedPtr & res);

  void waypoint_publish();

  int num_waypoints_published;

  std::vector<rosplane_msgs::msg::Waypoint> wps;
};

path_planner::path_planner()
    : Node("path_planner")
{

  waypoint_publisher = this->create_publisher<rosplane_msgs::msg::Waypoint>("waypoint_path", 10);

  next_waypoint_service = this->create_service<std_srvs::srv::Trigger>(
    "publish_next_waypoint",
    std::bind(&path_planner::publish_next_waypoint, this, _1, _2));
  
  add_waypoint_service = this->create_service<rosplane_msgs::srv::AddWaypoint>(
    "add_waypoint",
    std::bind(&path_planner::update_path, this, _1, _2));
  
  clear_waypoint_service = this->create_service<std_srvs::srv::Trigger>(
    "clear_waypoints",
    std::bind(&path_planner::clear_path, this, _1, _2));
  // TODO: Add load mission from file option

  timer_ = this->create_wall_timer(1000ms, std::bind(&path_planner::waypoint_publish, this));

  num_waypoints_published = 0;
}

path_planner::~path_planner() {}

bool path_planner::publish_next_waypoint(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                                         const std_srvs::srv::Trigger::Response::SharedPtr & res)
{

  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Publishing next waypoint, num_waypoints_published: " << num_waypoints_published);

  rosplane_msgs::msg::Waypoint new_waypoint = wps[num_waypoints_published];
  new_waypoint.clear_wp_list = false;

  waypoint_publisher->publish(new_waypoint);

  num_waypoints_published++;

  return true;
}

void path_planner::waypoint_publish()
{

  if (num_waypoints_published < NUM_WAYPOINTS_TO_PUBLISH_AT_START && num_waypoints_published < wps.size()) {

    rosplane_msgs::msg::Waypoint new_waypoint = wps[num_waypoints_published];
    new_waypoint.clear_wp_list = false;

    waypoint_publisher->publish(new_waypoint);

    num_waypoints_published++;
  }
}

bool path_planner::update_path(const rosplane_msgs::srv::AddWaypoint::Request::SharedPtr & req,
                               const rosplane_msgs::srv::AddWaypoint::Response::SharedPtr & res) {
  
  rosplane_msgs::msg::Waypoint new_waypoint;

  rclcpp::Time now = this->get_clock()->now();

  new_waypoint.header.stamp = now;
  new_waypoint.w = req->w;
  new_waypoint.chi_d = req->chi_d;
  new_waypoint.chi_valid = req->chi_valid;
  new_waypoint.va_d = req->va_d;
  new_waypoint.set_current = req->set_current;
  new_waypoint.clear_wp_list = false;

  if (req->publish_now) {
    wps.insert(wps.begin() + num_waypoints_published, new_waypoint);
    waypoint_publish();
    res->message = "Adding waypoint was successful! Waypoint published.";
  }
  else {
    wps.push_back(new_waypoint);
    res->message = "Adding waypoint was successful!";
  }

  res->success = true;
  
  return true;
}

bool path_planner::clear_path(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                              const std_srvs::srv::Trigger::Response::SharedPtr & res) {
  wps.clear();

  res->success = true;
  return true;
}

} // namespace rosplane

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rosplane::path_planner>();

  rclcpp::spin(node);

  return 0;
}
