/**
 * @file controller_base.h
 *
 * Base class definition for autopilot controller in chapter 6 of UAVbook, see http://uavbook.byu.edu/doku.php
 * Implements ROS2 functionality and support for control algorithm.
 *
 * @author Ian Reid <iyr27@byu.edu>
 */

#ifndef CONTROLLER_BASE_H
#define CONTROLLER_BASE_H

#include <chrono>
#include <cstring>
#include <iostream>
#include <variant>
#include <rclcpp/rclcpp.hpp>
#include <rosflight_msgs/msg/command.hpp>
#include <rosplane_msgs/msg/controller_commands.hpp>
#include <rosplane_msgs/msg/controller_internals_debug.hpp>
#include <rosplane_msgs/msg/detail/controller_internals_debug__struct.hpp>
#include <rosplane_msgs/msg/state.hpp>
#include <param_manager.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace rosplane
{
/**
 * This defines the different portions of the control algorithm.
 */
enum class alt_zones
{
  TAKE_OFF, /**< In the take off zone where the aircraft gains speed and altitude */
  CLIMB, /**< In the climb zone the aircraft proceeds to commanded altitude without course change. */
  ALTITUDE_HOLD /**< In the altitude hold zone the aircraft keeps altitude and follows commanded course */
};

/**
 * This class implements all of the basic functionality of a controller interfacing with ROS2.
 */

class controller_base : public rclcpp::Node
{
public:
  /**
   * Constructor for ROS2 setup and parameter initialization.
   */
  controller_base();

protected:
  /**
   * This struct holds all of the inputs to the control algorithm.
   */
  struct input_s
  {
    float Ts;     /**< time step */
    float h;      /**< altitude */
    float va;     /**< airspeed */
    float phi;    /**< roll angle */
    float theta;  /**< pitch angle */
    float chi;    /**< course angle */
    float p;      /**< body frame roll rate */
    float q;      /**< body frame pitch rate */
    float r;      /**< body frame yaw rate */
    float Va_c;   /**< commanded airspeed (m/s) */
    float h_c;    /**< commanded altitude (m) */
    float chi_c;  /**< commanded course (rad) */
    float phi_ff; /**< feed forward term for orbits (rad) */
  };

  /**
   * This struct holds all of the outputs of the control algorithm.
   */
  struct output_s
  {
    float theta_c;          /**< The commanded pitch angle from the altitude control loop */
    float phi_c;            /**< The commanded roll angle from the course control loop */
    float delta_e;          /**< The commanded elevator deflection */
    float delta_a;          /**< The commanded aileron deflection */
    float delta_r;          /**< The commanded rudder deflection */
    float delta_t;          /**< The commanded throttle deflection */
    alt_zones current_zone; /**< The current altitude zone for the control */
  };

  /**
   * Parameter manager object. Contains helper functions to interface parameters with ROS.
  */
  param_manager params;

  /**
   * Interface for control algorithm.
   * @param input Inputs to the control algorithm.
   * @param output Outputs of the controller, including selected intermediate values and final control efforts.
   */
  virtual void control(const struct input_s & input,
                       struct output_s & output) = 0;

  /**
   * The override for the intermediate values for the controller.
   */
  rosplane_msgs::msg::ControllerInternalsDebug
    tuning_debug_override_msg_; // TODO find a better and more permanent place for this.

private:
  /**
   * This publisher publishes the final calculated control surface deflections.
   */
  rclcpp::Publisher<rosflight_msgs::msg::Command>::SharedPtr actuators_pub_;

  /**
   * This publisher publishes the intermediate commands in the control algorithm.
   */
  rclcpp::Publisher<rosplane_msgs::msg::ControllerInternalsDebug>::SharedPtr internals_pub_;

  /**
   * This subscriber subscribes to the commands the controller uses to calculate control effort.
   */
  rclcpp::Subscription<rosplane_msgs::msg::ControllerCommands>::SharedPtr controller_commands_sub_;

  /**
   * This subscriber subscribes to the current state of the aircraft.
   */
  rclcpp::Subscription<rosplane_msgs::msg::State>::SharedPtr vehicle_state_sub_;

  /**
   * Subscribes to the tuning messages for later use in the control calculations
  */
  rclcpp::Subscription<rosplane_msgs::msg::ControllerInternalsDebug>::SharedPtr tuning_debug_sub_;

  /**
   * This timer controls how often commands are published by the autopilot.
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /** Parameters for use in control loops.*/
  /** Note that these parameters are not used:
   * 
   * double p_ff;                 //< pitch feedforward
   * double trim_r;               //< trim value for rudder
   * double max_r;                //< maximum value for rudder
   */
  std::map<std::string, std::variant<double, bool, int64_t, std::string>> params_;   // Can I cast ROS int to int?

  /**
   * The stored value for the most up to date commands for the controller.
   */
  rosplane_msgs::msg::ControllerCommands controller_commands_;

  /**
   * The stored value for the most up to date vehicle state (pose).
   */
  rosplane_msgs::msg::State vehicle_state_;

  /**
   * Flag to indicate if the first command has been received.
   */
  bool command_recieved_;

  /**
   * Convert from deflection angle in radians to pwm.
   */
  void convert_to_pwm(struct output_s & output);

  /**
   * Calls the control function and publishes outputs and intermediate values to the command and controller internals
   * topics.
   */
  void actuator_controls_publish();

  /**
   * Callback for new set of controller commands published to the controller_commands_sub_.
   * This saves the message as the member variable controller_commands_ for use in control loops.
   * @param msg ControllerCommands message.
   */
  void controller_commands_callback(const rosplane_msgs::msg::ControllerCommands::SharedPtr msg);

  /**
   * Callback for the new state of the aircraft published to the vehicle_state_sub_.
   * This saves the message as the member variable vehicle_state_ for sue in control loops.
   * @param msg
   */
  void vehicle_state_callback(const rosplane_msgs::msg::State::SharedPtr msg);

  /**
   * Callback for the overrided intermediate values of the controller interface for tuning.
   * This saves the message as the member variable tuing_debug_override_msg_.
   * @param msg
   */
  void tuning_debug_callback(const rosplane_msgs::msg::ControllerInternalsDebug::SharedPtr msg);

  /**
   * ROS2 parameter system interface. This connects ROS2 parameters with the defined update callback, parametersCallback.
   */
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  /**
   * Callback for when parameters are changed using ROS2 parameter system.
   * This takes all new changed params and updates the appropiate parameters in the params_ object.
   * @param parameters Set of updated parameters.
   * @return Service result object that tells the requester the result of the param update.
   */
  rcl_interfaces::msg::SetParametersResult
  parametersCallback(const std::vector<rclcpp::Parameter> & parameters);

  /**
   * This declares each parameter as a parameter so that the ROS2 parameter system can recognize each parameter.
   * It also sets the default parameter, which will then be overridden by a launch script.
   */
  void declare_parameters();

  /**
   * This creates a wall timer that calls the controller publisher.
  */
  void set_timer();
};
} // namespace rosplane

#endif // CONTROLLER_BASE_H
