/**
 * @file estimator_ros.h
 *
 * ROS-interface class definition for autopilot estimator in chapter 8 of UAVbook, see http://uavbook.byu.edu/doku.php
 *
 * Based on orignal work by Gary Ellingson.
 *
 * @author Ian Reid <ian.reid@byu.edu>
 */

#ifndef ESTIMATOR_ROS_H
#define ESTIMATOR_ROS_H

#include <chrono>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosflight_msgs/msg/airspeed.hpp>
#include <rosflight_msgs/msg/barometer.hpp>
#include <rosflight_msgs/msg/status.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <yaml-cpp/yaml.h>

#include "param_manager.hpp"
#include "rosplane_msgs/msg/state.hpp"

#define EARTH_RADIUS 6378145.0f

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace rosplane
{

class EstimatorROS : public rclcpp::Node
{
public:
  EstimatorROS();

protected:
  struct Input
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

  struct Output
  {
    float pn;
    float pe;
    float h;
    float va;
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

  virtual void estimate(const Input & input, Output & output) = 0;

  ParamManager params_;
  bool gps_init_;
  double init_lat_ = 0.0; /**< Initial latitude in degrees */
  double init_lon_ = 0.0; /**< Initial longitude in degrees */
  float init_alt_ = 0.0;  /**< Initial altitude in meters above MSL  */
  float init_static_;     /**< Initial static pressure (mbar)  */

private:
  rclcpp::Publisher<rosplane_msgs::msg::State>::SharedPtr vehicle_state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_fix_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    gnss_vel_sub_; //used in conjunction with the gnss_fix_sub_
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::Barometer>::SharedPtr baro_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::Airspeed>::SharedPtr airspeed_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::Status>::SharedPtr status_sub_;

  std::string param_filepath_ = "estimator_params.yaml";

  void update();
  void gnssFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void gnssVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void baroAltCallback(const rosflight_msgs::msg::Barometer::SharedPtr msg);
  /**
   * @brief This saves parameters to the param file for later use.
   *
   * @param param_name The name of the parameter.
   * @param param_val The value of the parameter.
   */
  void saveParameter(std::string param_name, double param_val);
  void airspeedCallback(const rosflight_msgs::msg::Airspeed::SharedPtr msg);
  void statusCallback(const rosflight_msgs::msg::Status::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr update_timer_;
  std::chrono::microseconds update_period_;
  bool params_initialized_;
  std::string gnss_fix_topic_ = "navsat_compat/fix";
  std::string gnss_vel_topic_ = "navsat_compat/vel";
  std::string imu_topic_ = "imu/data";
  std::string baro_topic_ = "baro";
  std::string airspeed_topic_ = "airspeed";
  std::string status_topic_ = "status";

  bool gps_new_;
  bool armed_first_time_;                 /**< Arm before starting estimation  */
  int baro_count_;                        /**< Used to grab the first set of baro measurements */
  std::vector<float> init_static_vector_; /**< Used to grab the first set of baro measurements */

  /**
   * This declares each parameter as a parameter so that the ROS2 parameter system can recognize each parameter.
   * It also sets the default parameter, which will then be overridden by a launch script.
   */
  void declare_parameters();

  /**
   * @brief Determines the period of a timer rosd on the ROS2 parameter and starts it 
   */
  void set_timer();

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

  Input input_;
};

} // namespace rosplane

#endif // ESTIMATOR_ROS_H
