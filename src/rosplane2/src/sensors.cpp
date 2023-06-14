#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rosflight_msgs/msg/barometer.hpp>
#include <rosflight_msgs/msg/airspeed.hpp>
#include <rosflight_msgs/msg/status.hpp>
#include "rosplane2_msgs/msg/state.hpp"
#include "Eigen/Geometry"
#include "random"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <geometry_msgs/msg/wrench.hpp>
#include "chrono"

#define EARTH_RADIUS 6378145.0f

using namespace std::chrono_literals;
using std::placeholders::_1;

// TODO split into .h and .cpp like the rest. Add into namespace.

class Sensors : public rclcpp::Node
{
public:
    Sensors()
            : Node("sensors") {

        gnss_fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(gnss_fix_topic_, 10);
        gnss_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(gnss_vel_topic_, 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
        baro_pub_ = this->create_publisher<rosflight_msgs::msg::Barometer>(baro_topic_, 10);
        airspeed_pub_ = this->create_publisher<rosflight_msgs::msg::Airspeed>(airspeed_topic_, 10);
        status_pub_ = this->create_publisher<rosflight_msgs::msg::Status>(status_topic_, 10);

        forces_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>("forces_moments", 10, std::bind(&Sensors::update_forces, this, std::placeholders::_1));
        state_sub_ = this->create_subscription<rosplane2_msgs::msg::State>("state", 10, std::bind(&Sensors::update_sensors, this, std::placeholders::_1));

        gps_sim_t_ = 1/gps_hz_;

        gnss_fix_.status.status = sensor_msgs::msg::NavSatStatus ::STATUS_NO_FIX;

    }

private:
    rclcpp::Subscription<rosplane2_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr forces_sub_;

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_fix_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr gnss_vel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<rosflight_msgs::msg::Barometer>::SharedPtr baro_pub_;
    rclcpp::Publisher<rosflight_msgs::msg::Airspeed>::SharedPtr airspeed_pub_;
    rclcpp::Publisher<rosflight_msgs::msg::Status>::SharedPtr status_pub_;

    rclcpp::TimerBase::SharedPtr pub_timer_;

    double update_rate_ = 100.0;
    std::string gnss_fix_topic_ = "navsat_compat/fix"; // TODO implement param callback.
    std::string gnss_vel_topic_ = "navsat_compat/vel";
    std::string imu_topic_ = "imu/data";
    std::string baro_topic_ = "baro";
    std::string airspeed_topic_ = "airspeed";
    std::string status_topic_ = "status";

    float mass_ = 11.0; // TODO implement param callbacks.
    float gravity_ = 9.8;
    float rho_ = 1.225;
    float gps_hz_ = 5;
    float sim_t_ = .01; // Time of simulation in seconds.
    float gps_k = 1. / 1100.;
    float gps_n_sigma = 0.01;
    float gps_e_sigma = 0.01;
    float gps_h_sigma = 0.03; // TODO move to the params_ struct
    float gps_Vg_sigma = 0.005;
    float gps_course_sigma = gps_Vg_sigma / 20.0;
    double gps_init_lat = 40.2669751;
    double gps_init_long = -111.6362489;
    double gps_init_alt = 1387.0;

    float accel_sigma = .0025*9.81; // std dev in m/s^2
    float gyro_sigma = M_PI*(.13)/180.0; // radians(.13); // std dev in rad/sec
    float abs_pres_sigma = 0.01*1000; // std dev in Pa
    float diff_pres_sigma = 0.002*1000; // std dev in Pa

    // GPS sim variables.
    float t_gps_ = 0;
    float gps_sim_t_;
    float gps_eta_n;
    float gps_eta_e;
    float gps_eta_h;
    float gps_n = 0.0;
    float gps_e = 0.0;
    float gps_h = 0.0;



    sensor_msgs::msg::NavSatFix gnss_fix_;
    geometry_msgs::msg::TwistStamped gnss_vel_;
    sensor_msgs::msg::Imu imu_;
    rosflight_msgs::msg::Barometer baro_;
    rosflight_msgs::msg::Airspeed airspeed_;
    rosflight_msgs::msg::Status status_;
    geometry_msgs::msg::Wrench forces_moments;

    bool first_time = true; // TODO this is a little clunky, find a better way to do this.



    void update_sensors(const rosplane2_msgs::msg::State::SharedPtr msg){


        status_.armed = true;

        std::random_device rd;

        std::default_random_engine generator (rd());

        std::normal_distribution<float> accel_dist(0.0, accel_sigma);
        std::normal_distribution<float> gyro_dist(0.0, gyro_sigma);
        std::normal_distribution<float> abs_pres_dist(0.0, abs_pres_sigma);
        std::normal_distribution<float> diff_pres_dist(0.0, diff_pres_sigma);
        std::normal_distribution<float> gps_n_dist(0.0, gps_n_sigma);
        std::normal_distribution<float> gps_e_dist(0.0, gps_e_sigma);
        std::normal_distribution<float> gps_h_dist(0.0, gps_h_sigma);
        std::normal_distribution<float> gps_Vg_dist(0.0, gps_Vg_sigma);

        imu_.angular_velocity.x = msg->p + gyro_dist(generator);
        imu_.angular_velocity.y = msg->q + gyro_dist(generator);
        imu_.angular_velocity.z = msg->r + gyro_dist(generator);

        imu_.linear_acceleration.x = forces_moments.force.x / mass_ + sin(msg->theta)*gravity_ + accel_dist(generator);
        imu_.linear_acceleration.y = forces_moments.force.y / mass_ - cos(msg->theta)*sin(msg->phi)*gravity_ + accel_dist(generator);
        imu_.linear_acceleration.z = forces_moments.force.z / mass_ - cos(msg->theta)*cos(msg->phi)*gravity_ + accel_dist(generator);

        baro_.pressure = rho_*gravity_*(-msg->position[2]) + abs_pres_dist(generator);
        baro_.altitude = -msg->position[2];

        airspeed_.differential_pressure = 0.5 * rho_ * pow(msg->va, 2) + diff_pres_dist(generator);

        if(t_gps_ >= gps_sim_t_){

            gps_eta_n = exp(-gps_k*gps_sim_t_)*gps_eta_n + gps_n_dist(generator);
            gps_eta_e = exp(-gps_k*gps_sim_t_)*gps_eta_e + gps_e_dist(generator);
            gps_eta_h = exp(-gps_k*gps_sim_t_)*gps_eta_h + gps_h_dist(generator);

            gps_n = msg->position[0] + gps_eta_n; // TODO add etas
            gps_e = msg->position[1] + gps_eta_e;
            gps_h = -msg->position[2] + gps_eta_h;

            gnss_fix_.latitude = 180.0/(M_PI * EARTH_RADIUS)*gps_n + gps_init_lat;
            gnss_fix_.longitude = 180.0/(M_PI * EARTH_RADIUS) * 1/cos(gps_init_lat*M_PI/180.0)*gps_e + gps_init_long;
            gnss_fix_.altitude = gps_h + gps_init_alt;

            gnss_fix_.header.stamp = this->get_clock()->now();

            gnss_fix_.status.status = sensor_msgs::msg::NavSatStatus ::STATUS_NO_FIX;

            if (gnss_fix_.latitude > .001 || gnss_fix_.longitude != .001) {
                gnss_fix_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
            }

            Eigen::Quaternionf quaternion;
            quaternion.w() = msg->quat[0];
            quaternion.x() = msg->quat[1];
            quaternion.y() = msg->quat[2];
            quaternion.z() = msg->quat[3];

            Eigen::Matrix<float, 3, 1> vels;

            vels << msg->u, msg->v, msg->w;

            Eigen::Matrix3f rot = quaternion.toRotationMatrix();

            Eigen::Matrix<float, 3, 1> pdot = rot * vels;

            gnss_vel_.twist.linear.x = pdot[0] + gps_Vg_dist(generator);
            gnss_vel_.twist.linear.y = pdot[1] + gps_Vg_dist(generator);

            t_gps_ = 0;

        } else {

            t_gps_ += gps_sim_t_;

        }

        pub_sensors();

        if (first_time){
            status_.armed = true;
            status_pub_->publish(status_);
        }

    }

    void update_forces(const geometry_msgs::msg::Wrench::SharedPtr msg){

        forces_moments = *msg;

    }

    void pub_sensors(){

        gnss_fix_pub_->publish(gnss_fix_);
        gnss_vel_pub_->publish(gnss_vel_);
        imu_pub_->publish(imu_);
        baro_pub_->publish(baro_);
        airspeed_pub_->publish(airspeed_);

    }

};

int main(int argc, char * argv[])
{
    rclcpp:: init(argc, argv);
    rclcpp::spin(std::make_shared<Sensors>());
    rclcpp::shutdown();
    return 0;
}
