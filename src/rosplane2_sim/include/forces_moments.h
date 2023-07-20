/*
 * Copyright 2016 James Jackson, MAGICC Lab, Brigham Young University - Provo, UT
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROSPLANE2_FORCES_MOMENTS_H
#define ROSPLANE2_FORCES_MOMENTS_H

#include <stdio.h>

#include <boost/bind/bind.hpp>
#include <eigen3/Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>

#include <rosplane2_msgs/msg/command.hpp>
#include <rosplane2_msgs/msg/state.hpp>
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "cmath"



//#include "rosplane_sim/gz_compat.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace rosplane2
{
//    static const std::string kDefaultWindSpeedSubTopic = "gazebo/wind_speed";


class AircraftForcesAndMoments : public rclcpp::Node
    {
    public:
        AircraftForcesAndMoments();


    private:
        void UpdateForcesAndMoments();
        void Reset();

        std::string command_topic_;
        std::string wind_speed_topic_;
        std::string joint_name_;
        std::string link_name_;
        std::string parent_frame_id_;
        std::string motor_speed_pub_topic_; //!!!
        std::string namespace_;

        // physical parameters
        double mass_ = 11.0;
        double Jx_ = .8244;
        double Jy_ = 1.135;
        double Jz_ = 1.759;
        double Jxz_ = .1204;
        double rho_ = 1.268;

        // motor and prop parameters

        float D_prop = 20*.0254; // prop diameter in meters.

        float KV_rpm_per_volt = 145.0;
        float KV = (1/KV_rpm_per_volt) * 60. / (2*M_PI); // Moving to correct units.
        float KQ = KV; // these are the same value when in the correct units.
        float R_motor = .042; // resistance of the motor.
        float i_0 = 1.5; // no load current draw.

        float number_cells = 12.; // number of cells in the battery.
        float V_max = number_cells*3.7; // maximum voltage that can be applied.

        // prop coeffecients.
        float C_Q2 = -0.01664;
        float C_Q1 = 0.004970;
        float C_Q0 = 0.005230;
        float C_T2 = -0.1079;
        float C_T1 = -0.06044;
        float C_T0 = 0.09357;

        // aerodynamic coefficients
        struct WingCoeff
        {
            double S;
            double b;
            double c;
            double M;
            double epsilon;
            double alpha0;
        } wing_ = {0.55, 2.8956, 0.18994, 50.0, 0.159, 0.47};

        // Propeller Coefficients
        struct PropCoeff
        {
            double k_motor;
            double k_T_P;
            double k_Omega;
            double e;
            double S;
            double C;
        } prop_ = {40.0, 0.0, 0.0, 0.8, 0.0314, 1.0};

        // Lift Coefficients
        struct LiftCoeff
        {
            double O;
            double alpha;
            double beta;
            double p;
            double q;
            double r;
            double delta_a;
            double delta_e;
            double delta_r;
        };

        LiftCoeff CL_ = {0.23, 5.61, 0.0, 0.0, 7.95, .0, .0, 0.13, .0};
        LiftCoeff CD_ = {0.043, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0, 0.134, 0.0};
        LiftCoeff Cm_ = {0.0135, -2.74, 0.0, 0.0, -38.21, 0.0, 0.0, -0.99, 0.0};
        LiftCoeff CY_ = {0.0, 0.00, -0.98, -0.07278, 0.0, 0.0, 0.075, 0.0, 0.19};
        LiftCoeff Cell_ = {0.0, 0.00, -0.13, -0.51, 0.0, 0.25, 0.17, 0.0, 0.0024};
        LiftCoeff Cn_{0.0, 0.0, 0.08557, 0.069, 0.0, -0.095, -0.0003, 0.0, -0.069}; // Cn_delta_a -.0011

        // not constants
        // actuators
        struct Actuators
        {
            double e;
            double a;
            double r;
            double t;
        } delta_;

        // wind
        struct Wind
        {
            double N;
            double E;
            double D;
        } wind_;

        // container for forces
        struct ForcesAndTorques
        {
            double Fx;
            double Fy;
            double Fz;
            double l;
            double m;
            double n;
        } forces_;

        double u;
        double v;
        double w;
        double p;
        double q;
        double r;

        // Time Counters
        double sampling_time_ = 0.001; // TODO update to have the ignition sampling time. Is this in seconds????
        double prev_sim_time_ = 0.001;

        // For reset handling
//        GazeboPose initial_pose_;

        rclcpp::Subscription<rosplane2_msgs::msg::Command>::SharedPtr command_sub_;
        rclcpp::Subscription<rosplane2_msgs::msg::State>::SharedPtr state_sub_;

        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr forces_moments_pub_;

        rclcpp::TimerBase::SharedPtr timer_;

        rosplane2_msgs::msg::State _state;

        Eigen::Quaternionf quat;

//        ros::Subscriber wind_speed_sub_;
//        rclcpp::Subscription<rosplane2_msgs::msg::State>::SharedPtr wind_speed_sub_;

//        boost::thread callback_queue_thread_;
//        void WindSpeedCallback(const geometry_msgs::Vector3 &wind);
        void CommandCallback(const rosplane2_msgs::msg::Command::SharedPtr msg);

        void StateCallback(const rosplane2_msgs::msg::State::SharedPtr msg);

        void SendForces();

        Eigen::Matrix<float, 2, 1> MotorThrustAndTorque(float Va, float delta_t);

//        GazeboVector wind_speed_W_;
    };
}

#endif //ROSPLANE2_FORCES_MOMENTS_H
