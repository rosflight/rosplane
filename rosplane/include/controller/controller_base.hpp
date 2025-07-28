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

#include <rclcpp/rclcpp.hpp>
#include <rosflight_msgs/msg/command.hpp>

#include "param_manager.hpp"
#include "rosplane_msgs/msg/controller_commands.hpp"
#include "rosplane_msgs/msg/controller_internals.hpp"
#include "rosplane_msgs/msg/state.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace rosplane
{
/**
 * This defines the different portions of the control algorithm.
 */
enum class AltZones
{
  TAKE_OFF, /**< In the take off zone where the aircraft gains speed and altitude */
  CLIMB, /**< In the climb zone the aircraft proceeds to commanded altitude without course change. */
  ALTITUDE_HOLD /**< In the altitude hold zone the aircraft keeps altitude and follows commanded course */
};

/**
 * This class implements all of the basic functionality of a controller interfacing with ROS2.
 */

class ControllerBase : public rclcpp::Node
{
public:
  /**
   * Constructor for ROS2 setup and parameter initialization.
   */
  ControllerBase();

  /**
   * Gets the current phi_c value from the current private command message.
   *
   * @return The latest phi_c value in the controller commands message, as received by the
   * ROS callback.
   */
  float get_phi_c() { return controller_commands_.phi_c; }

  /**
   * Gets the current theta_c value from the current private command message.
   *
   * @return The latest theta_c value in the controller commands message, as received by the
   * ROS callback.
   */
  float get_theta_c() { return controller_commands_.theta_c; };

protected:
  /**
   * This struct holds all of the inputs to the control algorithm.
   */
  struct Input
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
    float va_c;   /**< commanded airspeed (m/s) */
    float h_c;    /**< commanded altitude (m) */
    float chi_c;  /**< commanded course (rad) */
    float phi_ff; /**< feed forward term for orbits (rad) */
  };

  /**
   * This struct holds all of the outputs of the control algorithm.
   */
  struct Output
  {
    float theta_c;         /**< The commanded pitch angle from the altitude control loop */
    float phi_c;           /**< The commanded roll angle from the course control loop */
    float delta_e;         /**< The commanded elevator deflection */
    float delta_a;         /**< The commanded aileron deflection */
    float delta_r;         /**< The commanded rudder deflection */
    float delta_t;         /**< The commanded throttle deflection */
    AltZones current_zone; /**< The current altitude zone for the control */
  };

  /**
   * Parameter manager object. Contains helper functions to interface parameters with ROS.
  */
  ParamManager params_;

  /**
   * Interface for control algorithm.
   * @param input Inputs to the control algorithm.
   * @param output Outputs of the controller, including selected intermediate values and final control efforts.
   */
  virtual void control(const Input & input, Output & output) = 0;

private:
  /**
   * This publisher publishes the final calculated control surface deflections.
   */
  rclcpp::Publisher<rosflight_msgs::msg::Command>::SharedPtr actuators_pub_;

  /**
   * This publisher publishes the current commands in the control algorithm.
   */
  rclcpp::Publisher<rosplane_msgs::msg::ControllerInternals>::SharedPtr controller_internals_pub_;

  /**
   * This subscriber subscribes to the commands the controller uses to calculate control effort.
   */
  rclcpp::Subscription<rosplane_msgs::msg::ControllerCommands>::SharedPtr controller_commands_sub_;

  /**
   * This subscriber subscribes to the current state of the aircraft.
   */
  rclcpp::Subscription<rosplane_msgs::msg::State>::SharedPtr vehicle_state_sub_;

  /**
   * This timer controls how often commands are published by the autopilot.
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * Period of the timer that controlls how often commands are published.
   */
  std::chrono::microseconds timer_period_;

  /**
   * Flag that determines when params have been initialized to prevent errors when setting the timer
   */
  bool params_initialized_;

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
  void convert_to_pwm(Output & output);

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
