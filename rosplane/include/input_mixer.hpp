#ifndef INPUT_MIXER_HPP
#define INPUT_MIXER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rosflight_msgs/msg/rc_raw.hpp>
#include <rosplane_msgs/msg/controller_commands.hpp>
#include <param_manager.hpp>

namespace rosplane
{
class input_mixer : public rclcpp::Node
{
public:
  /**
   * @class input_mixer
   * @brief Class for input mixing.
   *
   * The input_mixer class is responsible for mixing various input commands, such as controller
   * commands and RC raw signals.
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
   * This function is called when a new message of type rosplane_msgs::msg::ControllerCommands is
   * received.
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

  /// Parameters stuff

  /**
   * Param manager object, for getting parameters from ROS.
   */
  param_manager params_;

  /**
   * ROS2 parameter system interface. This connects ROS2 parameters with the defined update callback,
   * parametersCallback.
   */
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  /**
   * Callback for when parameters are changed using ROS2 parameter system.
   * This takes all new changed params and updates the appropriate parameters in the params_ object.
   * @param parameters Set of updated parameters.
   * @return Service result object that tells the requester the result of the param update.
   */
  rcl_interfaces::msg::SetParametersResult
  parametersCallback(const std::vector<rclcpp::Parameter> & parameters);

};
}

#endif //INPUT_MIXER_HPP
