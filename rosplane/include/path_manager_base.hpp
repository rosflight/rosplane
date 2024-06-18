/**
 * @file path_manager_base.hpp
 *
 * Base class definition for autopilot path follower in chapter 10 of UAVbook, see http://uavbook.byu.edu/doku.php
 *
 * @author Gary Ellingson <gary.ellingson@byu.edu>
 * adapted by Judd Mehr and Brian Russel for ROSplane software
 */

#ifndef PATH_MANAGER_BASE_H
#define PATH_MANAGER_BASE_H

#include <Eigen/Eigen>
#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <rosplane_msgs/msg/current_path.hpp>
#include <rosplane_msgs/msg/state.hpp> // src/rosplane_msgs/msg/State.msg
#include <rosplane_msgs/msg/waypoint.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <param_manager.hpp>
//#include <rosplane/ControllerConfig.hpp>!!!
using std::placeholders::_1;
using namespace std::chrono_literals;

namespace rosplane
{
class path_manager_base : public rclcpp::Node
{
public:
  path_manager_base();

protected:

  std::vector<rosplane_msgs::msg::Waypoint> waypoints_;
  int num_waypoints_;
  int idx_a_; /** index to the waypoint that was most recently achieved */

  bool temp_waypoint_ = false;

  struct input_s
  {
    float pn;  /** position north */
    float pe;  /** position east */
    float h;   /** altitude */
    float chi; /** course angle */
  };

  struct output_s
  {
    bool flag;    /** Inicates strait line or orbital path (true is line, false is orbit) */
    float va_d;   /** Desired airspeed (m/s) */
    float r[3];   /** Vector to origin of straight line path (m) */
    float q[3];   /** Unit vector, desired direction of travel for line path */
    float c[3];   /** Center of orbital path (m) */
    float rho;    /** Radius of orbital path (m) */
    int8_t lamda; /** Direction of orbital path (cw is 1, ccw is -1) */
  };

  param_manager params;   /** Holds the parameters for the path_manager and children */

  virtual void manage(const struct input_s & input,
                      struct output_s & output) = 0;

private:
  rclcpp::Subscription<rosplane_msgs::msg::State>::SharedPtr
    vehicle_state_sub_; /**< vehicle state subscription */
  rclcpp::Subscription<rosplane_msgs::msg::Waypoint>::SharedPtr
    new_waypoint_sub_; /**< new waypoint subscription */
  rclcpp::Publisher<rosplane_msgs::msg::CurrentPath>::SharedPtr
    current_path_pub_; /**< controller commands publication */

  rosplane_msgs::msg::State vehicle_state_; /**< vehicle state */

  int64_t update_rate_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  void vehicle_state_callback(const rosplane_msgs::msg::State & msg);
  bool state_init_;
  void new_waypoint_callback(const rosplane_msgs::msg::Waypoint & msg);
  void current_path_publish();

  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rcl_interfaces::msg::SetParametersResult
  parametersCallback(const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Declares parameters with ROS2 and adds it to the parameter manager object
   */
  void declare_parameters();
};
} // namespace rosplane
#endif // PATH_MANAGER_BASE_H
