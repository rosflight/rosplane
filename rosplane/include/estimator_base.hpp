/**
 * @file estimator_base.h
 *
 * Base class definition for autopilot estimator in chapter 8 of UAVbook, see http://uavbook.byu.edu/doku.php
 *
 * @author Gary Ellingson <gary.ellingson@byu.edu>
 */

#ifndef ESTIMATOR_BASE_H
#define ESTIMATOR_BASE_H

#include <Eigen/Eigen>
#include <chrono>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <math.h>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <rosflight_msgs/msg/airspeed.hpp>
#include <rosflight_msgs/msg/barometer.hpp>
#include <rosflight_msgs/msg/status.hpp>
#include <rosplane_msgs/msg/state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <param_manager.hpp>

#define EARTH_RADIUS 6378145.0f

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace rosplane
{

class estimator_base : public rclcpp::Node
{
public:
  estimator_base();

protected:
  struct input_s
  {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float static_pres;
    float diff_pres;
    bool gps_new;
    float gps_n;
    float gps_e;
    float gps_h;
    float gps_Vg;
    float gps_course;
    bool status_armed;
    bool armed_init;
  };

  struct output_s
  {
    float pn;
    float pe;
    float h;
    float Va;
    float alpha;
    float beta;
    float phi;
    float theta;
    float psi;
    float chi;
    float p;
    float q;
    float r;
    float Vg;
    float wn;
    float we;
  };

  bool baro_init_; /**< Initial barometric pressure */

  virtual void estimate(const struct input_s & input,
                        struct output_s & output) = 0;

protected:
  param_manager params;

private:
  rclcpp::Publisher<rosplane_msgs::msg::State>::SharedPtr vehicle_state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_fix_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    gnss_vel_sub_; //used in conjunction with the gnss_fix_sub_
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::Barometer>::SharedPtr baro_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::Airspeed>::SharedPtr airspeed_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::Status>::SharedPtr status_sub_;

  void update();
  void gnssFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void gnssVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void baroAltCallback(const rosflight_msgs::msg::Barometer::SharedPtr msg);
  void airspeedCallback(const rosflight_msgs::msg::Airspeed::SharedPtr msg);
  void statusCallback(const rosflight_msgs::msg::Status::SharedPtr msg);

  // double update_rate_ = 100.0;
  rclcpp::TimerBase::SharedPtr update_timer_;
  std::string gnss_fix_topic_ = "navsat_compat/fix";
  std::string gnss_vel_topic_ = "navsat_compat/vel";
  std::string imu_topic_ = "imu/data";
  std::string baro_topic_ = "baro";
  std::string airspeed_topic_ = "airspeed";
  std::string status_topic_ = "status";

  bool gps_new_;
  bool gps_init_;
  double init_lat_ = 0.0;                 /**< Initial latitude in degrees */
  double init_lon_ = 0.0;                 /**< Initial longitude in degrees */
  float init_alt_ = 0.0;                  /**< Initial altitude in meters above MSL  */
  bool armed_first_time_;                 /**< Arm before starting estimation  */
  float init_static_;                     /**< Initial static pressure (mbar)  */
  int baro_count_;                        /**< Used to grab the first set of baro measurements */
  std::vector<float> init_static_vector_; /**< Used to grab the first set of baro measurements */

  /**
   * This declares each parameter as a parameter so that the ROS2 parameter system can recognize each parameter.
   * It also sets the default parameter, which will then be overridden by a launch script.
   */
  void declare_parameters();

  struct input_s input_;
};

} // namespace rosplane

#endif // ESTIMATOR_BASE_H
