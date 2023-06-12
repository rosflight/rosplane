/**
 * @file estimator_base.h
 *
 * Base class definition for autopilot estimator in chapter 8 of UAVbook, see http://uavbook.byu.edu/doku.php
 *
 * @author Gary Ellingson <gary.ellingson@byu.edu>
 */

#ifndef ESTIMATOR_BASE_H
#define ESTIMATOR_BASE_H

#include <rclcpp/rclcpp.hpp>
#include <rosplane2_msgs/msg/state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rosflight_msgs/msg/barometer.hpp>
#include <rosflight_msgs/msg/airspeed.hpp>
#include <rosflight_msgs/msg/status.hpp>
#include <math.h>
#include <Eigen/Eigen>
#include <numeric>
#include <chrono>

#define EARTH_RADIUS 6378145.0f

using std::placeholders::_1;

namespace rosplane2
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

  struct params_s
  {
    double gravity;
    double rho;
    double sigma_accel;
    double sigma_n_gps;
    double sigma_e_gps;
    double sigma_Vg_gps;
    double sigma_course_gps;
    double Ts;
  };

  virtual void estimate(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

private:
  rclcpp::Publisher<rosplane2_msgs::msg::State>::SharedPtr vehicle_state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_fix_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr gnss_vel_sub_; //used in conjunction with the gnss_fix_sub_
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

  double update_rate_ = 100.0;
  rclcpp::TimerBase::SharedPtr update_timer_;
  std::string gnss_fix_topic_ = "navsat_compat/fix";
  std::string gnss_vel_topic_ = "navsat_compat/vel";
  std::string imu_topic_ = "imu/data";
  std::string baro_topic_ = "baro";
  std::string airspeed_topic_ = "airspeed";
  std::string status_topic_ = "status";

  bool gps_new_;
  bool gps_init_;
  double init_lat_ = 0.0;       /**< Initial latitude in degrees */
  double init_lon_ = 0.0;       /**< Initial longitude in degrees */
  float init_alt_ = 0.0;        /**< Initial altitude in meters above MSL  */
  bool armed_first_time_; /**< Arm before starting estimation  */
  bool baro_init_;        /**< Initial barometric pressure */
  float init_static_;     /**< Initial static pressure (mbar)  */
  int baro_count_;        /**< Used to grab the first set of baro measurements */
  std::vector<float> init_static_vector_; /**< Used to grab the first set of baro measurements */

  struct params_s params_ = {9.8, 1.225, .0245, .21, .21, .0500, .0045, 1.0f/update_rate_ * 100.0};



  struct input_s input_;
};

} //end namespace

#endif // ESTIMATOR_BASE_H
