#include "rclcpp/rclcpp.hpp"
#include "rosplane_msgs/msg/waypoint.hpp"
#include "rosplane_msgs/srv/add_waypoint.hpp"
#include "rosflight_msgs/srv/param_file.hpp"
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/utilities.hpp>
// #include <rosplane_msgs/msg/detail/waypoint__struct.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <param_manager.hpp>
#include <yaml-cpp/yaml.h>

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

  param_manager params;   /** Holds the parameters for the path_planner*/

private:
  rclcpp::Publisher<rosplane_msgs::msg::Waypoint>::SharedPtr waypoint_publisher;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr next_waypoint_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_waypoint_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr print_waypoint_service;
  rclcpp::Service<rosplane_msgs::srv::AddWaypoint>::SharedPtr add_waypoint_service;
  rclcpp::Service<rosflight_msgs::srv::ParamFile>::SharedPtr load_mission_service;
  rclcpp::TimerBase::SharedPtr timer_;

  bool publish_next_waypoint(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                             const std_srvs::srv::Trigger::Response::SharedPtr & res);
  
  bool update_path(const rosplane_msgs::srv::AddWaypoint::Request::SharedPtr & req,
                   const rosplane_msgs::srv::AddWaypoint::Response::SharedPtr & res);
  
  bool clear_path(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                  const std_srvs::srv::Trigger::Response::SharedPtr & res);

  bool print_path(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                  const std_srvs::srv::Trigger::Response::SharedPtr & res);

  bool load_mission(const rosflight_msgs::srv::ParamFile::Request::SharedPtr & req,
                    const rosflight_msgs::srv::ParamFile::Response::SharedPtr & res);
  
  bool load_mission_from_file(const std::string& filename);

  void waypoint_publish();
  void timer_callback();
  /**
   * This declares each parameter as a parameter so that the ROS2 parameter system can recognize each parameter.
   * It also sets the default parameter, which will then be overridden by a launch script.
   */
  void declare_parameters();

  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rcl_interfaces::msg::SetParametersResult
  parametersCallback(const std::vector<rclcpp::Parameter> & parameters);

  int num_waypoints_published;

  std::vector<rosplane_msgs::msg::Waypoint> wps;
};

path_planner::path_planner()
    : Node("path_planner"), params(this)
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

  print_waypoint_service = this->create_service<std_srvs::srv::Trigger>(
    "print_waypoints",
    std::bind(&path_planner::print_path, this, _1, _2));

  load_mission_service = this->create_service<rosflight_msgs::srv::ParamFile>(
    "load_mission_from_file",
    std::bind(&path_planner::load_mission, this, _1, _2));

  // Set the parameter callback, for when parameters are changed.
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&path_planner::parametersCallback, this, std::placeholders::_1));
  declare_parameters();
  params.set_parameters();

  timer_ = this->create_wall_timer(1000ms, std::bind(&path_planner::timer_callback, this));

  num_waypoints_published = 0;
}

path_planner::~path_planner() {}

void path_planner::timer_callback() {
  if (num_waypoints_published < NUM_WAYPOINTS_TO_PUBLISH_AT_START && num_waypoints_published < (int) wps.size()) {
    waypoint_publish();
  }
}

bool path_planner::publish_next_waypoint(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                                         const std_srvs::srv::Trigger::Response::SharedPtr & res)
{

  if (num_waypoints_published < (int) wps.size()) {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Publishing next waypoint, num_waypoints_published: " << num_waypoints_published);

    waypoint_publish();

    res->success = true;
    return true;
  }
  else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "No waypoints left to publish! Add more waypoints");
    res->success = false;
    return false;
  }
}

void path_planner::waypoint_publish()
{

  rosplane_msgs::msg::Waypoint new_waypoint = wps[num_waypoints_published];
  new_waypoint.clear_wp_list = false;

  waypoint_publisher->publish(new_waypoint);

  num_waypoints_published++;
}

bool path_planner::update_path(const rosplane_msgs::srv::AddWaypoint::Request::SharedPtr & req,
                               const rosplane_msgs::srv::AddWaypoint::Response::SharedPtr & res) {
  
  rosplane_msgs::msg::Waypoint new_waypoint;

  rclcpp::Time now = this->get_clock()->now();

  new_waypoint.header.stamp = now;
  new_waypoint.w = req->w;
  new_waypoint.chi_d = req->chi_d;
  new_waypoint.use_chi = req->use_chi;
  new_waypoint.va_d = req->va_d;
  new_waypoint.set_current = req->set_current;

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

  rosplane_msgs::msg::Waypoint new_waypoint;
  new_waypoint.clear_wp_list = true;

  waypoint_publisher->publish(new_waypoint);

  res->success = true;
  return true;
}

bool path_planner::print_path(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                              const std_srvs::srv::Trigger::Response::SharedPtr & res) {
  std::stringstream output;
  
  output << "Printing waypoints...";

  for (int i=0; i < (int) wps.size(); ++i) {
    rosplane_msgs::msg::Waypoint wp = wps[i];
    output << std::endl << "----- WAYPOINT " << i << " -----" << std::endl;
    output << "Position (NED, meters): [" << wp.w[0] << ", " << wp.w[1] << ", " << wp.w[2] << "]" << std::endl;
    output << "Chi_d: " << wp.chi_d << " " << std::endl;
    output << "Va_d: " << wp.va_d << " " << std::endl;
    output << "use_chi: " << wp.use_chi;
  }

  // Print to info log stream
  RCLCPP_INFO_STREAM(this->get_logger(), output.str());

  res->success = true;

  return true;
}

bool path_planner::load_mission(const rosflight_msgs::srv::ParamFile::Request::SharedPtr & req,
                                const rosflight_msgs::srv::ParamFile::Response::SharedPtr & res) {
  // std::string filename = req->filename;
  res->success = load_mission_from_file(req->filename);
  return true;
}

bool path_planner::load_mission_from_file(const std::string& filename) {
  try {
    YAML::Node root = YAML::LoadFile(filename);
    assert(root.IsSequence());
    RCLCPP_INFO_STREAM(this->get_logger(), root);

    for (YAML::const_iterator it=root.begin(); it!=root.end(); ++it) {
      YAML::Node wp = it->second;

      rosplane_msgs::msg::Waypoint new_wp;
      new_wp.w = wp["w"].as<std::array<float, 3>>();
      new_wp.chi_d = wp["chi_d"].as<double>();
      new_wp.use_chi = wp["use_chi"].as<bool>();
      new_wp.va_d = wp["va_d"].as<double>();

      wps.push_back(new_wp);
    }

    return true;
  }
  catch (...) {
    return false;
  }
}

rcl_interfaces::msg::SetParametersResult
path_planner::parametersCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "One of the parameters given does not is not a parameter of the controller node.";

  bool success = params.set_parameters_callback(parameters);
  if (success)
  {
    result.successful = true;
    result.reason = "success";
  }

  return result;
}

void path_planner::declare_parameters() {
  params.declare_int("num_waypoints_to_publish_at_start", 3);
}

} // namespace rosplane

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rosplane::path_planner>();

  rclcpp::spin(node);

  return 0;
}
