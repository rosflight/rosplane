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

#include <rclcpp/rclcpp.hpp>
#include <rosplane_msgs/msg/detail/controller_internals_debug__struct.hpp>
#include <rosplane_msgs/msg/state.hpp>
#include <rosplane_msgs/msg/controller_commands.hpp>
#include <rosplane_msgs/msg/controller_internals_debug.hpp>
#include <rosflight_msgs/msg/command.hpp>
#include <chrono>
#include <iostream>
#include <cstring>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace rosplane
{
/**
 * This defines the different portions of the control algorithm.
 */
enum class alt_zones
{
  TAKE_OFF,                 /**< In the take off zone where the aircraft gains speed and altitude */
  CLIMB,                    /**< In the climb zone the aircraft proceeds to commanded altitude without course change. */
  ALTITUDE_HOLD             /**< In the altitude hold zone the aircraft keeps altitude and follows commanded course */
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
    float Ts;               /**< time step */
    float h;                /**< altitude */
    float va;               /**< airspeed */
    float phi;              /**< roll angle */
    float theta;            /**< pitch angle */
    float chi;              /**< course angle */
    float p;                /**< body frame roll rate */
    float q;                /**< body frame pitch rate */
    float r;                /**< body frame yaw rate */
    float Va_c;             /**< commanded airspeed (m/s) */
    float h_c;              /**< commanded altitude (m) */
    float chi_c;            /**< commanded course (rad) */
    float phi_ff;           /**< feed forward term for orbits (rad) */
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
   * This struct contains all of the parameters used by the controller.
   */
  struct params_s
  {
    double alt_hz;               /**< altitude hold zone */
    double alt_toz;              /**< altitude takeoff zone */
    double tau;                  /**< servo response time constant */
    double c_kp;                 /**< course hold proportional gain */
    double c_kd;                 /**< course hold derivative gain */
    double c_ki;                 /**< course hold integral gain */
    double r_kp;                 /**< roll hold proportional gain */
    double r_kd;                 /**< roll hold derivative gain */
    double r_ki;                 /**< roll hold integral gain */
    double p_kp;                 /**< pitch hold proportional gain */
    double p_kd;                 /**< pitch hold derivative gain */
    double p_ki;                 /**< pitch hold integral gain */
    double p_ff;                 /**< pitch feedforward */
    double a_t_kp;               /**< airspeed with throttle hold proportional gain */
    double a_t_kd;               /**< airspeed with throttle hold derivative gain */
    double a_t_ki;               /**< airspeed with throttle hold integral gain */
    double a_kp;                 /**< altitude hold proportional gain */
    double a_kd;                 /**< altitude hold derivative gain */
    double a_ki;                 /**< altitude hold integral gain */
    double l_kp;                 /**< energy balance proportional gain */
    double l_kd;                 /**< energy balance derivative gain */
    double l_ki;                 /**< energy balance integral gain */
    double e_kp;                 /**< total energy proportional gain */
    double e_kd;                 /**< total energy derivative gain */
    double e_ki;                 /**< total energy integral gain */
    double trim_e;               /**< trim value for elevator */
    double trim_a;               /**< trim value for aileron */
    double trim_r;               /**< trim value for rudder */
    double trim_t;               /**< trim value for throttle */
    double max_e;                /**< maximum value for elevator */
    double max_a;                /**< maximum value for aileron */
    double max_r;                /**< maximum value for rudder */
    double max_t;                /**< maximum value for throttle */
    double pwm_rad_e;            /**< conversion to pwm from radians for output of control loops */
    double pwm_rad_a;            /**< conversion to pwm from radians for output of control loops */
    double pwm_rad_r;            /**< conversion to pwm from radians for output of control loops */
    double max_takeoff_throttle; /**< maximum throttle allowed at takeoff */
    double mass;                 /**< mass of the aircraft */
    double gravity;              /**< gravity in m/s^2 */
    bool tuning_debug_override;
  };

  /**
   * Interface for control algorithm.
   * @param params Parameters used to calculate.
   * @param input Inputs to the control algorithm.
   * @param output Outputs of the controller, including selected intermediate values and final control efforts.
   */
  virtual void control(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

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

  rclcpp::Subscription<rosplane_msgs::msg::ControllerInternalsDebug>::SharedPtr tuning_debug_sub_;
  /**
   * This timer controls how often commands are published by the autopilot.
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /** Parameters for use in control loops.*/
  struct params_s params_ = {
          /* alt_hz */ 10.0,
          /* alt_toz */ 5.0,
          /* tau */ 50.0,
          /* c_kp */ 2.37,
          /* c_kd */ 0.0,
          /* c_ki */ 0.4,
          /* r_kp */ .06,
          /* r_kd */ 0.04,
          /* r_ki */ 0.0,
          /* p_kp */ -.15,
          /* p_kd */ -.05,
          /* p_ki */ 0.0,
          /* p_ff */ 0.0,
          /* a_t_kp */ .05,
          /* a_t_kd */ 0.0,
          /* a_t_ki */ .005,
          /* a_kp */ 0.015,
          /* a_kd */ 0.0,
          /* a_ki */ 0.003,
          /* l_kp */ 1.0,
          /* l_kd */ 0.0,
          /* l_ki */ 0.05,
          /* e_kp */ 5.0,
          /* e_kd */ 0.0,
          /* e_ki */ .9,
          /* trim_e */ 0.02,
          /* trim_a */ 0.0,
          /* trim_r */ 0.0,
          /* trim_t */ 0.5,
          /* max_e */ 0.61,
          /* max_a */ 0.15,
          /* max_r */ 0.523,
          /* max_t */ 1.0,
          /* pwm_rad_e */ 1.0,
          /* pwm_rad_a */ 1.0,
          /* pwm_rad_r */ 1.0,
          /* max_takeoff_throttle */ .55,
          /* mass */ 2.28,
          /* gravity */ 9.8,
          /* tuning_debug_override*/ false};

  /**
   * The stored value for the most up to date commands for the controller.
   */
  rosplane_msgs::msg::ControllerCommands controller_commands_;

  /**
   * The stored value for the most up to date vehicle state (pose).
   */
  rosplane_msgs::msg::State vehicle_state_;
  
  /**
   * The override for the intermediate values for the controller.
   */
  rosplane_msgs::msg::ControllerInternalsDebug tuning_debug_override_msg_;

  /**
   * Flag to indicate if the first command has been received.
   */
  bool command_recieved_;

  /**
   * Convert from deflection angle in radians to pwm.
   */
  void convert_to_pwm(struct output_s &output);

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
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

  /**
   * This declares each parameter as a parameter so that the ROS2 parameter system can recognize each parameter.
   */
  void declare_parameters();

  /**
   * This sets the parameters with the values in the params_ object from the supplied parameter file, or sets them to
   * the default if no value is given for a parameter.
   */
  void set_parameters();

};
} //end namespace

#endif // CONTROLLER_BASE_H
