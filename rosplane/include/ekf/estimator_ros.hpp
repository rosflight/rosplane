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
#include <filesystem>
#include <unordered_map>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosflight_msgs/msg/airspeed.hpp>
#include <rosflight_msgs/msg/barometer.hpp>
#include <rosflight_msgs/msg/gnss.hpp>
#include <rosflight_msgs/msg/status.hpp>
#include <rosplane_msgs/msg/state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include "param_manager/param_manager.hpp"

#define EARTH_RADIUS 6378145.0
#define NOT_IN_USE -1000000.0
#define MILLIS_TO_NANOS 1000000

namespace rosplane
{

class EstimatorROS : public rclcpp::Node
{
public:
  EstimatorROS();

protected:
  struct Input // FIXME: there are inputs that are not in this struct.
  {
    double gyro_x;
    double gyro_y;
    double gyro_z;
    double accel_x;
    double accel_y;
    double accel_z;
    bool baro_new;
    double static_pres;
    bool diff_new;
    double diff_pres;
    bool gps_new;
    int gps_yday;
    int gps_year;
    int gps_month;
    int gps_day;
    double gps_lat;
    double gps_lon;
    double gps_alt;
    double gps_n;
    double gps_e;
    double gps_h;
    double gps_vg;
    double gps_vn;
    double gps_ve;
    double gps_vd;
    double gps_course;
    bool status_armed;
    bool armed_init;
    bool mag_new;
    double mag_x;
    double mag_y;
    double mag_z;
  };

  struct Output
  {
    double pn = 0.0;
    double pe = 0.0;
    double pd = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    double h = 0.0;
    double va = 0.0;
    double alpha = 0.0;
    double beta = 0.0;
    double phi = 0.0;
    double theta = 0.0;
    double psi = 0.0;
    double bx = 0.0;
    double by = 0.0;
    double bz = 0.0;
    double chi = 0.0;
    double p = 0.0;
    double q = 0.0;
    double r = 0.0;
    double vg = 0.0;
    double wn = 0.0;
    double we = 0.0;
    Eigen::Quaterniond quat = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  };

  bool init_conds_saved_ = false;
  std::filesystem::path hotstart_path_;

  bool baro_init_ = false;

  double rho_;

  /**
   * @brief Indicates if the magnetometer magnetic field parameters have been initialized.
   */
  bool mag_init_ = false;

  virtual void estimate(const Input & input, Output & output) = 0;

  bool parameter_changed = false;

  ParamManager params_;
  bool gps_init_ = false;
  bool has_fix_ = false;
  double init_lat_ = 0.0; /**< Initial latitude in degrees */
  double init_lon_ = 0.0; /**< Initial longitude in degrees */
  double init_alt_ = 0.0; /**< Initial altitude in meters above MSL  */
  double init_static_;    /**< Initial static pressure (mbar)  */

  std::unordered_map<std::string, rclcpp::Time> time_since_last_sensor_update_;
  void set_sensor_monitoring();
  void check_sensors();

private:
  void hotstart();
  void saveInitConditions();

  rclcpp::Publisher<rosplane_msgs::msg::State>::SharedPtr vehicle_state_pub_;
  rclcpp::Subscription<rosflight_msgs::msg::GNSS>::SharedPtr gnss_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::Barometer>::SharedPtr baro_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::Airspeed>::SharedPtr airspeed_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::Status>::SharedPtr status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr magnetometer_sub_;

  void update();
  void gnssCallback(const rosflight_msgs::msg::GNSS::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void baroAltCallback(const rosflight_msgs::msg::Barometer::SharedPtr msg);
  void airspeedCallback(const rosflight_msgs::msg::Airspeed::SharedPtr msg);
  void update_barometer_calibration(const rosflight_msgs::msg::Barometer::SharedPtr msg);
  void statusCallback(const rosflight_msgs::msg::Status::SharedPtr msg);
  void magnetometerCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr update_timer_;
  std::chrono::microseconds update_period_;
  bool params_initialized_;
  std::string gnss_fix_topic_ = "gnss";
  std::string imu_topic_ = "imu/data";
  std::string baro_topic_ = "baro";
  std::string airspeed_topic_ = "airspeed";
  std::string status_topic_ = "status";
  std::string magnetometer_topic_ = "magnetometer";

  bool gps_new_;
  bool armed_first_time_;                  /**< Arm before starting estimation  */
  int baro_count_;                         /**< Used to grab the first set of baro measurements */
  std::vector<double> init_static_vector_; /**< Used to grab the first set of baro measurements */

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
