#ifndef INPUT_MAPPER_HPP
#define INPUT_MAPPER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rosflight_msgs/msg/rc_raw.hpp>
#include <rosplane_msgs/msg/controller_commands.hpp>
#include <param_manager.hpp>

namespace rosplane
{
class input_mapper : public rclcpp::Node
{
public:
  /**
   * @class input_mapper
   * @brief Class for input mixing.
   *
   * The input_mapper class is responsible for mixing various input commands, such as controller
   * commands and RC raw signals.
   */
  input_mapper();

private:
  /**
   * This bool keeps track of whether roll override is enabled.
   */
  bool roll_override_;
  /**
   * This bool keeps track of whether pitch override is enabled.
   */
  bool pitch_override_;

  /**
   * Bool value to keep track of whether a parameter change is pending.
   */
  bool param_change_pending_;

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
   * RC raw message, for storing the last RC input before being mixed and published.
   */
  rosflight_msgs::msg::RCRaw::SharedPtr rc_raw_msg_;

  /**
   * Service object for setting parameters of other nodes in autopilot.
   */
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_param_client_;

  /**
   * Timer for setting parameters of other nodes without ever blocking execution of this node.
   *
   * This timer exists so that waiting for the set param service and response will not block the
   * execution of the node, which needs to keep running to keep passing inputs around.
   */
  rclcpp::TimerBase::SharedPtr set_param_timer_;

  /**
   * Executor for setting parameters.
   */
  rclcpp::executors::SingleThreadedExecutor set_param_executor_;

  /**
   * Parameter request object, for setting parameters of other nodes while using alternate threads.
   */
  rcl_interfaces::srv::SetParameters::Request::SharedPtr set_param_request_;

  /**
   * FutureAndRequestId result for param set request.
   */
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture set_param_future_;

  /**
   * Helper function for knowing when to call a ROS service to change roll override.
   */
  void set_roll_override(bool roll_override);

  /**
   * Helper function for knowing when to call a ROS service to change pitch override.
   */
  void set_pitch_override(bool pitch_override);

  /**
   * Callback for set_param_timer_.
   */
  void set_param_timer_callback();

  /**
   * Sets a parameter in a different node over the ROS2 parameter system.
   */
  void set_external_param(std::string name, rclcpp::Parameter parameter);

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

#endif //INPUT_MAPPER_HPP
