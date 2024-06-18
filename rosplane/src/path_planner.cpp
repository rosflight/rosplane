#include "rclcpp/rclcpp.hpp"
#include "rosplane_msgs/msg/waypoint.hpp"
#include "rosplane_msgs/msg/state.hpp"
#include "rosplane_msgs/srv/add_waypoint.hpp"
#include "rosflight_msgs/srv/param_file.hpp"
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/utilities.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <param_manager.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>

#define EARTH_RADIUS 6378145.0f

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
  rclcpp::Publisher<rosplane_msgs::msg::Waypoint>::SharedPtr waypoint_publisher_;
  rclcpp::Subscription<rosplane_msgs::msg::State>::SharedPtr state_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr next_waypoint_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_waypoint_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr print_waypoint_service_;
  rclcpp::Service<rosplane_msgs::srv::AddWaypoint>::SharedPtr add_waypoint_service_;
  rclcpp::Service<rosflight_msgs::srv::ParamFile>::SharedPtr load_mission_service_;

  /**
   * @brief "publish_next_waypoint" service callback. Publish the next waypoint from the internal vector of waypoint objects. Will not publish if there are no more waypoints in the vector.
   * 
   * @param req: Pointer to a std_srvs::srv::Trigger request object
   * @param res: Pointer to a std_srvs::srv::Trigger response object
   * 
   * @return True if waypoint was published, false if no more waypoints were available to publish
   */
  bool publish_next_waypoint(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                             const std_srvs::srv::Trigger::Response::SharedPtr & res);

  /**
   * @brief "add_waypoint" service callback. Adds a waypoint to the end of the vector of waypoint objects and optionally publishes another waypoint.
   * 
   * @param req: Pointer to an AddWaypoint service request object
   * @param req: Pointer to an AddWaypoint service response object
   * 
   * @return True
   */
  bool update_path(const rosplane_msgs::srv::AddWaypoint::Request::SharedPtr & req,
                   const rosplane_msgs::srv::AddWaypoint::Response::SharedPtr & res);

  /**
   * @brief "clear_path" service callback. Clears all the waypoints internally and sends clear commands to path_manager
   * 
   * @param req: Pointer to a Trigger service request object
   * @param req: Pointer to a Trigger service response object
   * 
   * @return True
   */
  bool clear_path_callback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                  const std_srvs::srv::Trigger::Response::SharedPtr & res); void clear_path();

  /**
   * @brief "print_path" service callback. Prints waypoints to the terminal
   * 
   * @param req: Pointer to a Trigger service request object
   * @param req: Pointer to a Trigger service response object
   * 
   * @return True
   */
  bool print_path(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                  const std_srvs::srv::Trigger::Response::SharedPtr & res);

  /**
   * @brief "load_mission_from_file" service callback
   * 
   * @param req: Pointer to a ParamFile service request object
   * @param req: Pointer to a ParamFile service response object
   * 
   * @return True
   */
  bool load_mission(const rosflight_msgs::srv::ParamFile::Request::SharedPtr & req,
                    const rosflight_msgs::srv::ParamFile::Response::SharedPtr & res);

  /**
   * @brief Parses YAML file and loads waypoints
   * 
   * @param filename: String containing the path to the YAML file
   * 
   * @return True if loading waypoints was successful, false otherwise
   */
  bool load_mission_from_file(const std::string& filename);

  /**
   * @brief Callback for the rosplane_msgs::msg::State publisher. Saves the initial GNSS coordinates
   * 
   * @param msg: Pointer to a rosplane_msgs::msg::State object
   */
  void state_callback(const rosplane_msgs::msg::State & msg);

  /**
   * @brief Publishes the next waypoint in the vector of waypoints
   */
  void waypoint_publish();

  /**
   * @brief Publishes the number of initial waypoints given by parameter
   */
  void publish_initial_waypoints();

  /**
   * @brief Converts an LLA coordinate to NED coordinates
   * 
   * @param lla: Array of floats of size 3, with [latitude, longitude, altitude]
   * @return Array of doubles corresponding to the NED coordinates measured from the origin
   */
  std::array<double, 3> lla2ned(std::array<float, 3> lla);

  /**
   * @brief This declares each parameter as a parameter so that the ROS2 parameter system can recognize each parameter. It also sets the default parameter, which will then be overridden by a launch script.
   */
  void declare_parameters();

  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  /**
   * @brief Parameter change callback
   * 
   * @param parameters: Vector of rclcpp::Parameter that have been changed. 
   */
  rcl_interfaces::msg::SetParametersResult
  parametersCallback(const std::vector<rclcpp::Parameter> & parameters);

  int num_waypoints_published;
  double initial_lat_;
  double initial_lon_;
  double initial_alt_;

  std::vector<rosplane_msgs::msg::Waypoint> wps;
};

path_planner::path_planner()
    : Node("path_planner"), params(this)
{

  // Make this publisher transient_local so that it publishes the last 10 waypoints to late subscribers
  rclcpp::QoS qos_transient_local_10_(10);
  qos_transient_local_10_.transient_local();
  waypoint_publisher_ = this->create_publisher<rosplane_msgs::msg::Waypoint>("waypoint_path", qos_transient_local_10_);

  next_waypoint_service_ = this->create_service<std_srvs::srv::Trigger>(
    "publish_next_waypoint", std::bind(&path_planner::publish_next_waypoint, this, _1, _2));
  
  add_waypoint_service_ = this->create_service<rosplane_msgs::srv::AddWaypoint>(
    "add_waypoint", std::bind(&path_planner::update_path, this, _1, _2));
  
  clear_waypoint_service_ = this->create_service<std_srvs::srv::Trigger>(
    "clear_waypoints", std::bind(&path_planner::clear_path_callback, this, _1, _2));

  print_waypoint_service_ = this->create_service<std_srvs::srv::Trigger>(
    "print_waypoints", std::bind(&path_planner::print_path, this, _1, _2));

  load_mission_service_ = this->create_service<rosflight_msgs::srv::ParamFile>(
    "load_mission_from_file", std::bind(&path_planner::load_mission, this, _1, _2));
  
  state_subscription_ = this->create_subscription<rosplane_msgs::msg::State>("estimated_state", 10,
    std::bind(&path_planner::state_callback, this, _1));

  // Set the parameter callback, for when parameters are changed.
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&path_planner::parametersCallback, this, std::placeholders::_1));

  declare_parameters();
  params.set_parameters();

  num_waypoints_published = 0;

  // Initialize by publishing a clear path command.
  // This makes sure rviz or other vizualization tools don't show stale waypoints if rosplane is restarted.
  clear_path();

  // Publishes the initial waypoints
  publish_initial_waypoints();
}

path_planner::~path_planner() {}

void path_planner::publish_initial_waypoints() {
  int num_waypoints_to_publish_at_start = this->get_parameter("num_waypoints_to_publish_at_start").as_int();

  RCLCPP_INFO_STREAM(this->get_logger(), "Path Planner will publish the first {" << num_waypoints_to_publish_at_start << "} available waypoints!");

  while (num_waypoints_published < num_waypoints_to_publish_at_start && num_waypoints_published < (int) wps.size()) {
    waypoint_publish();
  }
}

void path_planner::state_callback(const rosplane_msgs::msg::State & msg) {
  // Make sure initial LLA is not zero before updating to avoid errors
  if (fabs(msg.initial_lat) > 0.0 || fabs(msg.initial_lon) > 0.0 || fabs(msg.initial_alt) > 0.0) {
    initial_lat_ = msg.initial_lat;
    initial_lon_ = msg.initial_lon;
    initial_alt_ = msg.initial_alt;
  }
}

bool path_planner::publish_next_waypoint(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                                         const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  if (num_waypoints_published < (int) wps.size()) {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Publishing next waypoint, num_waypoints_published: " << num_waypoints_published + 1);

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

  waypoint_publisher_->publish(new_waypoint);

  num_waypoints_published++;
}

bool path_planner::update_path(const rosplane_msgs::srv::AddWaypoint::Request::SharedPtr & req,
                               const rosplane_msgs::srv::AddWaypoint::Response::SharedPtr & res) {
  
  rosplane_msgs::msg::Waypoint new_waypoint;

  rclcpp::Time now = this->get_clock()->now();

  new_waypoint.header.stamp = now;
  
  // Convert to NED if given in LLA
  std::string lla_or_ned = "LLA";
  if (req->lla) {
    std::array<double, 3> ned = lla2ned(req->w);

    new_waypoint.w[0] = ned[0];
    new_waypoint.w[1] = ned[1];
    new_waypoint.w[2] = ned[2];
  }
  else {
    new_waypoint.w = req->w;
    lla_or_ned = "NED";
  }
  new_waypoint.chi_d = req->chi_d;
  new_waypoint.lla = req->lla;
  new_waypoint.use_chi = req->use_chi;
  new_waypoint.va_d = req->va_d;
  new_waypoint.set_current = req->set_current;

  if (req->publish_now) {
    wps.insert(wps.begin() + num_waypoints_published, new_waypoint);
    waypoint_publish();
    res->message = "Adding " + lla_or_ned + " waypoint was successful! Waypoint published.";
  }
  else {
    wps.push_back(new_waypoint);
    res->message = "Adding " + lla_or_ned + " waypoint was successful!";
  }

  res->success = true;
  return true;
}

bool path_planner::clear_path_callback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                              const std_srvs::srv::Trigger::Response::SharedPtr & res) {
  clear_path();

  res->success = true;
  return true;
}

void path_planner::clear_path() {
  wps.clear();

  rosplane_msgs::msg::Waypoint new_waypoint;
  new_waypoint.clear_wp_list = true;

  waypoint_publisher_->publish(new_waypoint);

  num_waypoints_published = 0;
}

bool path_planner::print_path(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                              const std_srvs::srv::Trigger::Response::SharedPtr & res) {
  std::stringstream output;
  
  output << "Printing waypoints...";

  for (int i=0; i < (int) wps.size(); ++i) {
    rosplane_msgs::msg::Waypoint wp = wps[i];
    output << std::endl << "----- WAYPOINT " << i << " -----" << std::endl;
    
    if (wp.lla) {
      output << "Position (LLA): [" << wp.w[0] << ", " << wp.w[1] << ", " << wp.w[2] << "]" << std::endl;
    }
    else {
      output << "Position (NED, meters): [" << wp.w[0] << ", " << wp.w[1] << ", " << wp.w[2] << "]" << std::endl;
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

bool path_planner::load_mission(const rosflight_msgs::srv::ParamFile::Request::SharedPtr & req,
                                const rosflight_msgs::srv::ParamFile::Response::SharedPtr & res) {
  clear_path();
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
  }
  catch (...) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Error while parsing mission YAML file! Check inputs");
    return false;
  }
}

std::array<double, 3> path_planner::lla2ned(std::array<float, 3> lla) {
  double lat1 = lla[0];
  double lon1 = lla[1];
  double alt1 = lla[2];

  double lat0 = initial_lat_;
  double lon0 = initial_lon_;
  double alt0 = initial_alt_;

  double n = EARTH_RADIUS * (lat1 - lat0) * M_PI / 180.0;
  double e = EARTH_RADIUS * cos(lat0 * M_PI / 180.0) * (lon1 - lon0) * M_PI / 180.0;
  double d = -(alt1 - alt0);

  if (fabs(initial_lat_) == 0.0 || fabs(initial_lon_) == 0.0 || fabs(initial_alt_) == 0.0) {
    RCLCPP_WARN_STREAM(this->get_logger(), "NED position set to [" << n << "," << e << "," << d << "]! Waypoints may be incorrect. Check GPS health");
  }

  return std::array<double, 3> {n, e, d};
}

rcl_interfaces::msg::SetParametersResult
path_planner::parametersCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "One of the parameters given is not a parameter of the controller node.";

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
