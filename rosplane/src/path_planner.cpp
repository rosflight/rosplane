#include <cmath>

#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/utilities.hpp>
#include <rosflight_msgs/srv/param_file.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>

#include "param_manager.hpp"
#include "rosplane_msgs/msg/state.hpp"
#include "rosplane_msgs/msg/waypoint.hpp"
#include "rosplane_msgs/srv/add_waypoint.hpp"

#include "path_planner.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace rosplane
{

PathPlanner::PathPlanner()
    : Node("path_planner")
    , params_(this)
{

  // Make this publisher transient_local so that it publishes the last 10 waypoints to late subscribers
  rclcpp::QoS qos_transient_local_10_(10);
  qos_transient_local_10_.transient_local();
  waypoint_publisher_ =
    this->create_publisher<rosplane_msgs::msg::Waypoint>("waypoint_path", qos_transient_local_10_);

  next_waypoint_service_ = this->create_service<std_srvs::srv::Trigger>(
    "publish_next_waypoint", std::bind(&PathPlanner::publish_next_waypoint, this, _1, _2));

  add_waypoint_service_ = this->create_service<rosplane_msgs::srv::AddWaypoint>(
    "add_waypoint", std::bind(&PathPlanner::update_path, this, _1, _2));

  clear_waypoint_service_ = this->create_service<std_srvs::srv::Trigger>(
    "clear_waypoints", std::bind(&PathPlanner::clear_path_callback, this, _1, _2));

  print_waypoint_service_ = this->create_service<std_srvs::srv::Trigger>(
    "print_waypoints", std::bind(&PathPlanner::print_path, this, _1, _2));

  load_mission_service_ = this->create_service<rosflight_msgs::srv::ParamFile>(
    "load_mission_from_file", std::bind(&PathPlanner::load_mission, this, _1, _2));

  state_subscription_ = this->create_subscription<rosplane_msgs::msg::State>(
    "estimated_state", 10, std::bind(&PathPlanner::state_callback, this, _1));

  // Set the parameter callback, for when parameters are changed.
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&PathPlanner::parametersCallback, this, std::placeholders::_1));

  // Declare parameters with ROS2 and save them to the param_manager object
  declare_parameters();
  params_.set_parameters();

  num_waypoints_published_ = 0;

  // Initialize by publishing a clear path command.
  // This makes sure rviz or other vizualization tools don't show stale waypoints if ROSplane is restarted.
  clear_path();

  // Publishes the initial waypoints
  publish_initial_waypoints();
}

PathPlanner::~PathPlanner() {}

void PathPlanner::publish_initial_waypoints()
{
  int num_waypoints_to_publish_at_start =
    this->get_parameter("num_waypoints_to_publish_at_start").as_int();

  RCLCPP_INFO_STREAM_ONCE(this->get_logger(),
                          "Path Planner will publish the first {"
                            << num_waypoints_to_publish_at_start << "} available waypoints!");

  // Publish the first waypoints as defined by the num_waypoints_to_publish_at_start parameter
  while (num_waypoints_published_ < num_waypoints_to_publish_at_start
         && num_waypoints_published_ < (int) wps.size()) {
    waypoint_publish();
  }
}

void PathPlanner::state_callback(const rosplane_msgs::msg::State & msg)
{
  // Make sure initial LLA is not zero before updating to avoid initialization errors
  // TODO: What if we want to initialize it at (0,0,0)?
  if (fabs(msg.initial_lat) > 0.0 || fabs(msg.initial_lon) > 0.0 || fabs(msg.initial_alt) > 0.0) {
    initial_lat_ = msg.initial_lat;
    initial_lon_ = msg.initial_lon;
    initial_alt_ = msg.initial_alt;
  }
}

bool PathPlanner::publish_next_waypoint(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                                        const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  // Publish the next waypoint, if available
  if (num_waypoints_published_ < (int) wps.size()) {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Publishing next waypoint, num_waypoints_published: " << num_waypoints_published_ + 1);

    waypoint_publish();

    res->success = true;
    return true;
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "No waypoints left to publish! Add more waypoints");
    res->success = false;
    return false;
  }
}

void PathPlanner::waypoint_publish()
{
  // Publish the next waypoint off the list
  rosplane_msgs::msg::Waypoint new_waypoint = wps[num_waypoints_published_];

  waypoint_publisher_->publish(new_waypoint);

  num_waypoints_published_++;
}

bool PathPlanner::update_path(const rosplane_msgs::srv::AddWaypoint::Request::SharedPtr & req,
                              const rosplane_msgs::srv::AddWaypoint::Response::SharedPtr & res)
{

  rosplane_msgs::msg::Waypoint new_waypoint;

  rclcpp::Time now = this->get_clock()->now();

  new_waypoint.header.stamp = now;

  // Create a string flag for the debug message (defaults to "LLA")
  std::string lla_or_ned = "LLA";

  // Convert to NED if given in LLA
  if (req->lla) {
    std::array<double, 3> ned = lla2ned(req->w);
    new_waypoint.w[0] = ned[0];
    new_waypoint.w[1] = ned[1];
    new_waypoint.w[2] = ned[2];
  } else {
    new_waypoint.w = req->w;
    lla_or_ned = "NED";
  }

  // Fill in the Waypoint object with the information from the service request object
  new_waypoint.chi_d = req->chi_d;
  new_waypoint.lla = req->lla;
  new_waypoint.use_chi = req->use_chi;
  new_waypoint.va_d = req->va_d;
  new_waypoint.set_current = req->set_current;

  if (req->publish_now) {
    // Insert the waypoint in the correct location in the list and publish it
    wps.insert(wps.begin() + num_waypoints_published_, new_waypoint);
    waypoint_publish();
    res->message = "Adding " + lla_or_ned + " waypoint was successful! Waypoint published.";
  } else {
    wps.push_back(new_waypoint);
    res->message = "Adding " + lla_or_ned + " waypoint was successful!";
  }

  publish_initial_waypoints();

  res->success = true;
  return true;
}

bool PathPlanner::clear_path_callback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                                      const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  clear_path();

  res->success = true;
  return true;
}

void PathPlanner::clear_path()
{
  wps.clear();

  // Publish a waypoint with "clear_wp_list" set to true to let downstream subscribers know they need to clear their waypoints
  rosplane_msgs::msg::Waypoint new_waypoint;
  new_waypoint.clear_wp_list = true;

  waypoint_publisher_->publish(new_waypoint);

  num_waypoints_published_ = 0;
}

bool PathPlanner::print_path(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                             const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  std::stringstream output;

  output << "Printing waypoints...";

  for (int i = 0; i < (int) wps.size(); ++i) {
    rosplane_msgs::msg::Waypoint wp = wps[i];
    output << std::endl << "----- WAYPOINT " << i << " -----" << std::endl;

    if (wp.lla) {
      output << "Position (LLA): [" << wp.w[0] << ", " << wp.w[1] << ", " << wp.w[2] << "]"
             << std::endl;
    } else {
      output << "Position (NED, meters): [" << wp.w[0] << ", " << wp.w[1] << ", " << wp.w[2] << "]"
             << std::endl;
    }
    output << "Chi_d: " << wp.chi_d << " " << std::endl;
    output << "Va_d: " << wp.va_d << " " << std::endl;
    output << "use_chi: " << wp.use_chi;
  }

  // Print to info log stream
  RCLCPP_INFO_STREAM(this->get_logger(), output.str());

  res->success = true;

  return true;
}

bool PathPlanner::load_mission(const rosflight_msgs::srv::ParamFile::Request::SharedPtr & req,
                               const rosflight_msgs::srv::ParamFile::Response::SharedPtr & res)
{
  clear_path();
  res->success = load_mission_from_file(req->filename);
  publish_initial_waypoints();
  return true;
}

bool PathPlanner::load_mission_from_file(const std::string & filename)
{
  try {
    YAML::Node root = YAML::LoadFile(filename);
    assert(root.IsSequence());
    RCLCPP_INFO_STREAM(this->get_logger(), root);

    for (YAML::const_iterator it = root.begin(); it != root.end(); ++it) {
      YAML::Node wp = it->second;

      rosplane_msgs::msg::Waypoint new_wp;
      new_wp.w = wp["w"].as<std::array<float, 3>>();

      // If LLA, convert to NED
      if (wp["lla"].as<bool>()) {
        std::array<double, 3> ned = lla2ned(new_wp.w);

        new_wp.w[0] = ned[0];
        new_wp.w[1] = ned[1];
        new_wp.w[2] = ned[2];
      }

      new_wp.chi_d = wp["chi_d"].as<double>();
      new_wp.use_chi = wp["use_chi"].as<bool>();
      new_wp.va_d = wp["va_d"].as<double>();

      wps.push_back(new_wp);
    }

    return true;
  } catch (...) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Error while parsing mission YAML file! Check inputs");
    return false;
  }
}

std::array<double, 3> PathPlanner::lla2ned(std::array<float, 3> lla)
{
  double lat1 = lla[0];
  double lon1 = lla[1];
  double alt1 = lla[2];

  double lat0 = initial_lat_;
  double lon0 = initial_lon_;
  double alt0 = initial_alt_;

  double n = EARTH_RADIUS * (lat1 - lat0) * M_PI / 180.0;
  double e = EARTH_RADIUS * cos(lat0 * M_PI / 180.0) * (lon1 - lon0) * M_PI / 180.0;
  double d = -(alt1 - alt0);

  // Usually will not be flying exactly at these locations.
  // If the GPS reports (0,0,0), it most likely means there is an error with the GPS
  if (fabs(initial_lat_) == 0.0 || fabs(initial_lon_) == 0.0 || fabs(initial_alt_) == 0.0) {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "NED position set to ["
                         << n << "," << e << "," << d
                         << "]! Waypoints may be incorrect. Check GPS health");
  }

  return std::array<double, 3>{n, e, d};
}

rcl_interfaces::msg::SetParametersResult
PathPlanner::parametersCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "One of the parameters given is not a parameter of the controller node.";

  // Set parameters in the param_manager object
  bool success = params_.set_parameters_callback(parameters);
  if (success) {
    result.successful = true;
    result.reason = "success";
  }

  return result;
}

void PathPlanner::declare_parameters()
{
  params_.declare_int("num_waypoints_to_publish_at_start", 3);
}

} // namespace rosplane

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rosplane::PathPlanner>();

  rclcpp::spin(node);

  return 0;
}
