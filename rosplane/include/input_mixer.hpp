#ifndef INPUT_MIXER_HPP
#define INPUT_MIXER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rosflight_msgs/msg/rc_raw.hpp>
#include <rosplane_msgs/msg/controller_commands.hpp>

namespace rosplane
{
class input_mixer : public rclcpp::Node
{
public:
  /**
   * @class input_mixer
   * @brief Class for input mixing.
   *
   * The input_mixer class is responsible for mixing various input commands, such as controller commands and RC raw
   * signals.
   */
  input_mixer();

private:
  /**
   * This publisher publishes the mixed control commands.
   */
  rclcpp::Publisher<rosplane_msgs::msg::ControllerCommands>::SharedPtr mixed_commands_pub_;

  /**
   * This subscriber subscribes to the controller commands.
   */
  rclcpp::Subscription<rosplane_msgs::msg::ControllerCommands>::SharedPtr controller_commands_sub_;
  /**
   * This subscriber subscribes to the RC raw signals.
   */
  rclcpp::Subscription<rosflight_msgs::msg::RCRaw>::SharedPtr rc_raw_sub_;

  /**
   * This function is called when a new message of type rosplane_msgs::msg::ControllerCommands is received.
   *
   * @param msg A shared pointer to the received message.
   */
  void controller_commands_callback(const rosplane_msgs::msg::ControllerCommands::SharedPtr msg);
  /**
   * This function is called when a new message of type `rosflight_msgs::msg::RCRaw` is received.
   *
   * @param msg A shared pointer to the received message.
   */
  void rc_raw_callback(const rosflight_msgs::msg::RCRaw::SharedPtr msg);

};
}

#endif //INPUT_MIXER_HPP
