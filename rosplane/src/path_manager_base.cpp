#include "path_manager_base.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "iostream"
#include "path_manager_example.hpp"
#include "rclcpp/rclcpp.hpp"
#include <limits>
#include <rclcpp/logging.hpp>

namespace rosplane
{

path_manager_base::path_manager_base()
    : Node("rosplane_path_manager"), params_(this), params_initialized_(false)
{
  vehicle_state_sub_ = this->create_subscription<rosplane_msgs::msg::State>(
    "estimated_state", 10, std::bind(&path_manager_base::vehicle_state_callback, this, _1));
  new_waypoint_sub_ = this->create_subscription<rosplane_msgs::msg::Waypoint>(
    "waypoint_path", 10, std::bind(&path_manager_base::new_waypoint_callback, this, _1));
  current_path_pub_ = this->create_publisher<rosplane_msgs::msg::CurrentPath>("current_path", 10);

  // Set the parameter callback, for when parameters are changed.
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&path_manager_base::parametersCallback, this, std::placeholders::_1));

  // Declare parameters maintained by this node with ROS2. Required for all ROS2 parameters associated with this node
  declare_parameters();
  params_.set_parameters();

  params_initialized_ = true;

  // Now that the update rate has been updated in parameters, create the timer
  set_timer();

  num_waypoints_ = 0;

  state_init_ = false;
}

void path_manager_base::declare_parameters() {
  params_.declare_double("R_min", 50.0);
  params_.declare_double("current_path_pub_frequency", 100.0);
}

void path_manager_base::set_timer() {
  // Calculate the period in milliseconds from the frequency
  double frequency = params_.get_double("current_path_pub_frequency");
  timer_period_ = std::chrono::microseconds(static_cast<long long>(1.0 / frequency * 1e6));

  update_timer_ =
    this->create_wall_timer(timer_period_, std::bind(&path_manager_base::current_path_publish, this));
}

rcl_interfaces::msg::SetParametersResult
path_manager_base::parametersCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "One of the parameters given is not a parameter of the controller node.";

  // Update the changed parameters in the param_manager object
  bool success = params_.set_parameters_callback(parameters);
  if (success)
  {
    result.successful = true;
    result.reason = "success";
  }

  // If the frequency parameter was changed, restart the timer.
  if (params_initialized_ && success) {
    double frequency = params_.get_double("current_path_pub_frequency");
    std::chrono::microseconds curr_period = std::chrono::microseconds(static_cast<long long>(1.0 / frequency * 1e6));
    if (timer_period_ != curr_period) {
      update_timer_->cancel();
      set_timer();
    }
  }

  return result;
}

void path_manager_base::vehicle_state_callback(const rosplane_msgs::msg::State & msg)
{

  vehicle_state_ = msg;

  state_init_ = true;
}

void path_manager_base::new_waypoint_callback(const rosplane_msgs::msg::Waypoint & msg)
{
  double R_min = params_.get_double("R_min");
  orbit_dir = 0;

  // If the message contains "clear_wp_list", then clear all waypoints and do nothing else
  if (msg.clear_wp_list == true) {
    waypoints_.clear();
    num_waypoints_ = 0;
    idx_a_ = 0;
    return;
  }

  // If there are currently no waypoints in the list, then add a temporary waypoint as
  // the current state of the aircraft. This is necessary to define a line for line following.
  if (waypoints_.size() == 0)
  {
    waypoint_s temp_waypoint;

    temp_waypoint.w[0] = vehicle_state_.position[0];
    temp_waypoint.w[1] = vehicle_state_.position[1];
    temp_waypoint.w[2] = vehicle_state_.position[2];

    temp_waypoint.chi_d = 0.0; // Doesn't matter, it is never used.
    temp_waypoint.use_chi = false;
    temp_waypoint.va_d = msg.va_d; // Use the va_d for the next waypoint.

    waypoints_.push_back(temp_waypoint);
    num_waypoints_++;
    temp_waypoint_ = true;
  }
  
  // Add a default comparison for the last waypoint for feasiblity check.
  waypoint_s nextwp;
  Eigen::Vector3f w_existing(std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity());
  nextwp.w[0] = msg.w[0];
  nextwp.w[1] = msg.w[1];
  nextwp.w[2] = msg.w[2];
  nextwp.chi_d = msg.chi_d;
  nextwp.use_chi = msg.use_chi;
  nextwp.va_d = msg.va_d;

  // Save the last waypoint for comparison.
  if (waypoints_.size() > 0)
  {
    waypoint_s waypoint = waypoints_.back();
    w_existing << waypoint.w[0], waypoint.w[1], waypoint.w[2];
  }
  waypoints_.push_back(nextwp);
  num_waypoints_++;

  // Warn if too close to the last waypoint.
  Eigen::Vector3f w_new(msg.w[0], msg.w[1], msg.w[2]);

  if ((w_new - w_existing).norm() < R_min)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "A waypoint is too close to the next waypoint. Indices: " << waypoints_.size() - 2 << ", " << waypoints_.size() - 1);
  }
}

void path_manager_base::current_path_publish()
{

  struct input_s input;
  input.pn = vehicle_state_.position[0]; // position north
  input.pe = vehicle_state_.position[1]; // position east
  input.h = -vehicle_state_.position[2]; // altitude
  input.chi = vehicle_state_.chi;

  struct output_s output;

  if (state_init_ == true) {
    manage(input, output);
  }

  rosplane_msgs::msg::CurrentPath current_path;

  rclcpp::Time now = this->get_clock()->now();

  // Populate current_path message
  current_path.header.stamp = now;
  if (output.flag) {
    current_path.path_type = current_path.LINE_PATH;
  }
  else {
    current_path.path_type = current_path.ORBIT_PATH;
  }
  current_path.va_d = output.va_d;
  for (int i = 0; i < 3; i++) {
    current_path.r[i] = output.r[i];
    current_path.q[i] = output.q[i];
    current_path.c[i] = output.c[i];
  }
  current_path.rho = output.rho;
  current_path.lamda = output.lamda;

  current_path_pub_->publish(current_path);
}

} // namespace rosplane

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rosplane::path_manager_example>());

  return 0;
}
