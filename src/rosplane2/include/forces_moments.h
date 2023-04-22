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

//#include <gazebo/common/common.hh>
//#include <gazebo/common/Plugin.hh>
//#include <gazebo/gazebo.hh>
//#include <gazebo/physics/physics.hh>
//#include <rclcpp/callback_queue.h>
#include <rclcpp/rclcpp.hpp>

#include <rosplane2_msgs/msg/command.hpp>
#include <rosplane2_msgs/msg/state.hpp>
//#include <std_msgs/msg/Float32.hpp>
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/wrench.hpp"



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
//
//        physics::WorldPtr world_;
//        physics::ModelPtr model_;
//        physics::LinkPtr link_;
//        physics::JointPtr joint_;
//        physics::EntityPtr parent_link_;
//        event::ConnectionPtr updateConnection_; // Pointer to the update event connection.

        // physical parameters
        double mass_ = 3.92;
        double Jx_ = .213;
        double Jy_ = .171;
        double Jz_ = .350;
        double Jxz_ = .04;
        double rho_ = 1.268;

        // aerodynamic coefficients
        struct WingCoeff
        {
            double S;
            double b;
            double c;
            double M;
            double epsilon;
            double alpha0;
        } wing_ = {0.468, 1.8, 0.26, 50, 0.159, 0.304};

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

        LiftCoeff CL_ = {0.2869, 5.1378, 0.0, .0, 1.7102, .0, .0, 0.5202, .0};
        LiftCoeff CD_ = {0.03087, 0.0043021, 0.0, 0.02815, 0.2514, 0.0, 0.0, 0.01879, 0.0};
        LiftCoeff Cm_ = {0.0362, -0.2627, 0.0, 0.0, -9.7213, 0.0, 0.0, -1.2392, 0.0};
        LiftCoeff CY_ = {0.0, 0.00, -0.2471, -0.07278, 0.0, 0.1849, -0.02344, 0.0, 0.1591};
        LiftCoeff Cell_ = {0.0, 0.00, 0.0193, -0.5406, 0.0, 0.1929, 0.2818, 0.0, 0.00096};
        LiftCoeff Cn_{0.0, 0.0, 0.08557, -0.0498, 0.0, -0.0572, 0.0095, 0.0, -0.06};

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

//        ros::Subscriber wind_speed_sub_;
//        rclcpp::Subscription<rosplane2_msgs::msg::State>::SharedPtr wind_speed_sub_;

//        boost::thread callback_queue_thread_;
//        void WindSpeedCallback(const geometry_msgs::Vector3 &wind);
        void CommandCallback(const rosplane2_msgs::msg::Command::SharedPtr msg);

        void StateCallback(const rosplane2_msgs::msg::State::SharedPtr msg);

        void SendForces();

//        GazeboVector wind_speed_W_;
    };
}

#endif //ROSPLANE2_FORCES_MOMENTS_H
