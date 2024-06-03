#include "path_manager_base.hpp"
#include "iostream"
#include "path_manager_example.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logging.hpp>

namespace rosplane
{

path_manager_base::path_manager_base()
    : Node("rosplane_path_manager"), params(this)
{
  vehicle_state_sub_ = this->create_subscription<rosplane_msgs::msg::State>(
    "estimated_state", 10, std::bind(&path_manager_base::vehicle_state_callback, this, _1));
  new_waypoint_sub_ = this->create_subscription<rosplane_msgs::msg::Waypoint>(
    "waypoint_path", 10, std::bind(&path_manager_base::new_waypoint_callback, this, _1));
  current_path_pub_ = this->create_publisher<rosplane_msgs::msg::CurrentPath>("current_path", 10);
  update_timer_ =
    this->create_wall_timer(10ms, std::bind(&path_manager_base::current_path_publish, this));
  // interesting read on wall timer
  // https://answers.ros.org/question/375561/create-wall-timer-using-callback-with-parameters-ros2-c/
  //
  // Set the parameter callback, for when parameters are changed.
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&path_manager_base::parametersCallback, this, std::placeholders::_1));

  num_waypoints_ = 0;

  state_init_ = false;
}

rcl_interfaces::msg::SetParametersResult
path_manager_base::parametersCallback(const std::vector<rclcpp::Parameter> & parameters)
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

void path_manager_base::vehicle_state_callback(const rosplane_msgs::msg::State & msg)
{

  vehicle_state_ = msg;

  state_init_ = true;
}

void path_manager_base::new_waypoint_callback(const rosplane_msgs::msg::Waypoint & msg)
{
  if (msg.clear_wp_list == true) {
    waypoints_.clear();
    num_waypoints_ = 0;
    idx_a_ = 0;
    return;
  }

  waypoint_s nextwp;
  nextwp.w[0] = msg.w[0];
  nextwp.w[1] = msg.w[1];
  nextwp.w[2] = msg.w[2];
  nextwp.chi_d = msg.chi_d;
  nextwp.chi_valid = msg.chi_valid;
  nextwp.va_d = msg.va_d;
  waypoints_.push_back(nextwp);
  num_waypoints_++;
}

void path_manager_base::current_path_publish() //const rclcpp::TimerEvent &
{

  struct input_s input;
  input.pn = vehicle_state_.position[0]; // position north
  input.pe = vehicle_state_.position[1]; // position east
  input.h = -vehicle_state_.position[2]; // altitude
  input.chi = vehicle_state_.chi;

  struct output_s output;

  if (state_init_ == true) { manage(input, output); }

  rosplane_msgs::msg::CurrentPath current_path;

  rclcpp::Time now = this->get_clock()->now();

  current_path.header.stamp = now;

  if (output.flag) current_path.path_type = current_path.LINE_PATH;
  else
    current_path.path_type = current_path.ORBIT_PATH;
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
