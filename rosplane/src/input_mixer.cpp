#include "input_mixer.hpp"

using std::placeholders::_1;

namespace rosplane
{

input_mixer::input_mixer() : Node("input_mixer"), params_(this)
{
  mixed_commands_pub_ = this->create_publisher<rosplane_msgs::msg::ControllerCommands>(
    "mixed_commands", 10);
  controller_commands_sub_ = this->create_subscription<rosplane_msgs::msg::ControllerCommands>(
    "controller_commands", 10,
    std::bind(&input_mixer::controller_commands_callback, this, _1));
  rc_raw_sub_ = this->create_subscription<rosflight_msgs::msg::RCRaw>(
      "rc_raw", 10,
      std::bind(&input_mixer::rc_raw_callback, this, _1));

  /// Parameters stuff

  params_.declare_string("aileron_input", "path_follower");

  // Set the parameter callback, for when parameters are changed.
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&input_mixer::parametersCallback, this, _1));
}

void input_mixer::controller_commands_callback(const rosplane_msgs::msg::ControllerCommands::SharedPtr msg)
{
  mixed_commands_pub_->publish(*msg);

  if (params_.get_string("aileron_input") != "path_follower")
  {
    RCLCPP_WARN(this->get_logger(), "invalid parameter for aileron_input, setting to path_planner");
    params_.set_string("aileron_input", "path_follower");
  }
}

rcl_interfaces::msg::SetParametersResult
input_mixer::parametersCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  bool success = params_.set_parameters_callback(parameters);
  if (!success)
  {
    result.successful = false;
    result.reason =
      "One of the parameters given does not is not a parameter of the input mixer node.";
  }

  return result;
}

void input_mixer::rc_raw_callback(const rosflight_msgs::msg::RCRaw::SharedPtr msg)
{

}
} // namespace rosplane

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rosplane::input_mixer>();
  rclcpp::spin(node);
  return 0;
}