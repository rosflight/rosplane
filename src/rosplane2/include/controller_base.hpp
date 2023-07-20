/**
 * @file controller_base.h
 *
 * Base class definition for autopilot controller in chapter 6 of UAVbook, see http://uavbook.byu.edu/doku.php
 *
 * @author Gary Ellingson <gary.ellingson@byu.edu>
 */

#ifndef CONTROLLER_BASE_H
#define CONTROLLER_BASE_H

#include <rclcpp/rclcpp.hpp>
#include <rosplane2_msgs/msg/state.hpp>
#include <rosplane2_msgs/msg/controller_commands.hpp>
#include <rosplane2_msgs/msg/controller_internals.hpp>
#include "rosflight_msgs/msg/command.hpp"
#include "chrono"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace rosplane2
{

enum class alt_zones
{
  TAKE_OFF,
  CLIMB,
  DESCEND,
  ALTITUDE_HOLD
};

class controller_base : public rclcpp::Node
{
public:
  controller_base();
  float spin();

protected:

  struct input_s
  {
    float Ts;               /** time step */
    float h;                /** altitude */
    float va;               /** airspeed */
    float phi;              /** roll angle */
    float theta;            /** pitch angle */
    float chi;              /** course angle */
    float p;                /** body frame roll rate */
    float q;                /** body frame pitch rate */
    float r;                /** body frame yaw rate */
    float Va_c;             /** commanded airspeed (m/s) */
    float h_c;              /** commanded altitude (m) */
    float chi_c;            /** commanded course (rad) */
    float phi_ff;           /** feed forward term for orbits (rad) */
  };

  struct output_s
  {
    float theta_c;
    float delta_e;
    float phi_c;
    float delta_a;
    float delta_r;
    float delta_t;
    alt_zones current_zone;
  };

  struct params_s
  {
    double alt_hz;           /**< altitude hold zone */
    double alt_toz;          /**< altitude takeoff zone */
    double tau;
    double c_kp;
    double c_kd;
    double c_ki;
    double r_kp;
    double r_kd;
    double r_ki;
    double p_kp;
    double p_kd;
    double p_ki;
    double p_ff;
    double a_p_kp;
    double a_p_kd;
    double a_p_ki;
    double a_t_kp;
    double a_t_kd;
    double a_t_ki;
    double a_kp;
    double a_kd;
    double a_ki;
    double b_kp;
    double b_kd;
    double b_ki;
    double trim_e;
    double trim_a;
    double trim_r;
    double trim_t;
    double max_e;
    double max_a;
    double max_r;
    double max_t;
    double pwm_rad_e;
    double pwm_rad_a;
    double pwm_rad_r;
  };

  virtual void control(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

private:
  rclcpp::Publisher<rosflight_msgs::msg::Command>::SharedPtr actuators_pub_;
  rclcpp::Subscription<rosplane2_msgs::msg::ControllerCommands>::SharedPtr controller_commands_sub_;
  rclcpp::Subscription<rosplane2_msgs::msg::State>::SharedPtr vehicle_state_sub_;
  rclcpp::Subscription<rosplane2_msgs::msg::State>::SharedPtr vehicle_estimated_state_sub_;
  rclcpp::Publisher<rosplane2_msgs::msg::ControllerInternals>::SharedPtr internals_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

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
          /* p_kp */ -.15, // .-2
          /* p_kd */ -.05, // -.08
          /* p_ki */ 0.0,
          /* p_ff */ 0.0,
          /* a_p_kp */ -0.0713,
          /* a_p_kd */ -0.0635,
          /* a_p_ki */ 0.00,
          /* a_t_kp */ .05,
          /* a_t_kd */ 0.0,
          /* a_t_ki */ .005,
          /* a_kp */ 0.015, // .05
          /* a_kd */ 0.0,
          /* a_ki */ 0.003, // .001
          /* b_kp */ -0.1164,
          /* b_kd */ 0.0,
          /* b_ki */ -0.0037111,
          /* trim_e */ 0.02,
          /* trim_a */ 0.0,
          /* trim_r */ 0.0,
          /* trim_t */ 0.7,
          /* max_e */ 0.61,
          /* max_a */ 0.15,
          /* max_r */ 0.523,
          /* max_t */ 1.0,
          /* pwm_rad_e */ 1.0,
          /* pwm_rad_a */ 1.0,
          /* pwm_rad_r */ 1.0};           /**< params */

  rosplane2_msgs::msg::ControllerCommands controller_commands_;
  rosplane2_msgs::msg::State vehicle_state_;

  bool command_recieved_;

  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  /**
  * Convert from deflection angle to pwm
  */
  void convert_to_pwm(struct output_s &output);

  /**
  * Publish the outputs
  */
  void actuator_controls_publish();

  void controller_commands_callback(const rosplane2_msgs::msg::ControllerCommands::SharedPtr msg);

  void vehicle_state_callback(const rosplane2_msgs::msg::State::SharedPtr msg);

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

  void set_parameters();

  void declare_parameters();

};
} //end namespace

#endif // CONTROLLER_BASE_H
