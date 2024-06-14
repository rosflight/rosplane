#include "input_mixer.hpp"

using std::placeholders::_1;

namespace rosplane
{

input_mixer::input_mixer() : Node("input_mixer")
{
  mixed_commands_pub_ = this->create_publisher<rosplane_msgs::msg::ControllerCommands>(
    "mixed_commands", 10);
  controller_commands_sub_ = this->create_subscription<rosplane_msgs::msg::ControllerCommands>(
    "controller_commands", 10,
    std::bind(&input_mixer::controller_commands_callback, this, _1));
  rc_raw_sub_ = this->create_subscription<rosflight_msgs::msg::RCRaw>(
      "rc_raw", 10,
      std::bind(&input_mixer::rc_raw_callback, this, _1));
}

void input_mixer::controller_commands_callback(const rosplane_msgs::msg::ControllerCommands::SharedPtr msg)
{
  mixed_commands_pub_->publish(*msg);
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