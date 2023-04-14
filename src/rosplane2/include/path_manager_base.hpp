/**
 * @file path_manager_base.hpp
 *
 * Base class definition for autopilot path follower in chapter 10 of UAVbook, see http://uavbook.byu.edu/doku.php
 *
 * @author Gary Ellingson <gary.ellingson@byu.edu>
 * adapted by Judd Mehr and Brian Russel for RosPlane2 software
 */

#ifndef PATH_MANAGER_BASE_H
#define PATH_MANAGER_BASE_H

#include <rclcpp/rclcpp.hpp> 
#include <rosplane2_msgs/msg/state.hpp> // src/rosplane2_msgs/msg/State.msg
#include <rosplane2_msgs/msg/current_path.hpp>
#include <rosplane2_msgs/msg/waypoint.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <math.h>
#include <Eigen/Eigen>
//#include <rosplane2/ControllerConfig.hpp>

namespace rosplane2
{
class path_manager_base : public rclcpp::Node
{
public:
  path_manager_base();

protected:

  struct waypoint_s
  {
    float w[3];
    float chi_d;
    bool  chi_valid;
    float va_d;
  };

  std::vector<waypoint_s> waypoints_;
  int num_waypoints_;
  int idx_a_;                 /** index to the waypoint that was most recently achieved */

  struct input_s
  {
    float pn;               /** position north */
    float pe;               /** position east */
    float h;                /** altitude */
    float chi;              /** course angle */
  };

  struct output_s
  {
    bool  flag;             /** Inicates strait line or orbital path (true is line, false is orbit) */
    float va_d;             /** Desired airspeed (m/s) */
    float r[3];             /** Vector to origin of straight line path (m) */
    float q[3];             /** Unit vector, desired direction of travel for line path */
    float c[3];             /** Center of orbital path (m) */
    float rho;              /** Radius of orbital path (m) */
    int8_t lamda;          /** Direction of orbital path (cw is 1, ccw is -1) */
  };

  struct params_s
  {
    double R_min;
  };

  virtual void manage(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

private:

  //rclcpp::Node nh_;
  //rclcpp::Node nh_private_;
  //rclcpp::Subscriber vehicle_state_sub_;     /**< vehicle state subscription */
  //rclcpp::Subscriber new_waypoint_sub_;      /**< new waypoint subscription */
  //rclcpp::Publisher  current_path_pub_;      /**< controller commands publication */

  struct params_s params_;

  rosplane2_msgs::msg::State vehicle_state_;     /**< vehicle state */

  double update_rate_;
  rclcpp::Time update_timer_;

  void vehicle_state_callback(const rosplane2_msgs::msg::State &msg);
  bool state_init_;
  void new_waypoint_callback(const rosplane2_msgs::msg::Waypoint &msg);
  void current_path_publish(const rclcpp::TimerEvent &);
};
} //end namespace
#endif // PATH_MANAGER_BASE_H
