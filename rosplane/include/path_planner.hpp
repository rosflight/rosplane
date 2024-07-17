#include <rclcpp/executors.hpp>
#include <rclcpp/service.hpp>
#include <rosflight_msgs/srv/param_file.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "param_manager.hpp"
#include "rosplane_msgs/msg/state.hpp"
#include "rosplane_msgs/msg/waypoint.hpp"
#include "rosplane_msgs/srv/add_waypoint.hpp"

#define EARTH_RADIUS 6378145.0f

namespace rosplane
{

class PathPlanner : public rclcpp::Node
{
public:
  PathPlanner();
  ~PathPlanner();

  ParamManager params_; /** Holds the parameters for the path_planner*/

private:
  /**
   * Publishes waypoint objects
   */
  rclcpp::Publisher<rosplane_msgs::msg::Waypoint>::SharedPtr waypoint_publisher_;

  /**
   * Subscribes to Vehicle state
   */
  rclcpp::Subscription<rosplane_msgs::msg::State>::SharedPtr state_subscription_;

  /**
   * Service handle that publishes the next unpublished waypoint in the waypoint list
   */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr next_waypoint_service_;

  /**
   * Service handle that clears the waypoint list and publishes a waypoint with "clear_wp_list" set true
   */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_waypoint_service_;

  /**
   * Service handle that prints the loaded waypoints to the command line
   */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr print_waypoint_service_;

  /**
   * Service handle that adds a waypoint to the waypoint list
   */
  rclcpp::Service<rosplane_msgs::srv::AddWaypoint>::SharedPtr add_waypoint_service_;

  /**
   * Service handle that loads a list of waypoints (i.e., a mission) from a file
   */
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
                           const std_srvs::srv::Trigger::Response::SharedPtr & res);
  void clear_path();

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
  bool load_mission_from_file(const std::string & filename);

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

  int num_waypoints_published_;
  double initial_lat_;
  double initial_lon_;
  double initial_alt_;

  /**
   * Vector of waypoints
   */
  std::vector<rosplane_msgs::msg::Waypoint> wps;
};
} // namespace rosplane