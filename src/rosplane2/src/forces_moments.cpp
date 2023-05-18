/*
 * Copyright 2016 James Jackson, MAGICC Lab, Brigham Young University, Provo, UT
 * Copyright 2016 Gary Ellingson, MAGICC Lab, Brigham Young University, Provo, UT
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

#include "forces_moments.h"


namespace rosplane2
{
    AircraftForcesAndMoments::AircraftForcesAndMoments() : Node("forces_moments_pub") {
        // Initialize Wind
        wind_.N = 0.0;
        wind_.E = 0.0;
        wind_.D = 0.0;

        //initialize deltas
        delta_.t = 0.0;
        delta_.e = 0.0;
        delta_.a = 0.0;
        delta_.r = 0.0;

        forces_moments_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>("forces_moments",10);
        command_sub_ = this->create_subscription<rosplane2_msgs::msg::Command>(
                "command", 10, std::bind(&AircraftForcesAndMoments::CommandCallback, this, std::placeholders::_1));
        state_sub_ = this->create_subscription<rosplane2_msgs::msg::State>(
                "state", 10, std::bind(&AircraftForcesAndMoments::StateCallback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(10ms, std::bind(&AircraftForcesAndMoments::SendForces, this)); // TODO: update this to the publish every timestep.
    }

    void AircraftForcesAndMoments::StateCallback(const rosplane2_msgs::msg::State::SharedPtr msg)
    {
        u = msg->u;
        v = msg->v;
        w = msg->w;
        p = msg->p;
        q = msg->q;
        r = msg->r;

        quat.w() = msg->quat[0];
        quat.x() = msg->quat[1];
        quat.y() = msg->quat[2];
        quat.z() = msg->quat[3];

        _state = *msg;

    }

    void AircraftForcesAndMoments::CommandCallback(const rosplane2_msgs::msg::Command::SharedPtr msg)
    {
        // This is a little bit weird.  We need to nail down why these are negative
        delta_.t = msg->f;
        delta_.e = -msg->y;
        delta_.a = msg->x;
        delta_.r = -msg->z;
        UpdateForcesAndMoments();
    }

    void AircraftForcesAndMoments::UpdateForcesAndMoments()
    {
        // wind info is available in the wind_ struct
        /// TODO: This is wrong. Wind is being applied in the body frame, not inertial frame
        double ur = u - wind_.N;
        double vr = v - wind_.E;
        double wr = w - wind_.D;

        double Va = sqrt(pow(ur, 2.0) + pow(vr, 2.0) + pow(wr, 2.0));

        //gravity
        Eigen::Vector3f fg(0.0, 0.0, 9.81*mass_);

        Eigen::Matrix3f rotation = quat.toRotationMatrix();

        fg = rotation*fg;

        // Don't divide by zero, and don't let NaN's get through (sometimes GetRelativeLinearVel returns NaNs)
        if (Va > 0.000001 && std::isfinite(Va))
        {
            /*
             * The following math follows the method described in chapter 4 of
             * Small Unmanned Aircraft: Theory and Practice
             * By Randy Beard and Tim McLain.
             * Look there for a detailed explanation of each line in the rest of this function
             */

            double alpha;

            if (ur == 0){
                alpha = std::copysign(1, wr) * M_PI/2.0;
            }
            else {
                 alpha= atan2(wr, ur);
            }

            double beta = asin(vr/Va);

            float tmp = std::sqrt(pow(ur,2) + pow(wr,2));
            if (tmp == 0){
                beta = std::copysign(1, vr)*M_PI/2.0;
            }

            double sign = (alpha >= 0 ? 1 : -1); //Sigmoid function
            double sigma_a = (1 + exp(-(wing_.M*(alpha - wing_.alpha0))) + exp((wing_.M*(alpha + wing_.alpha0))))/((1 + exp(-
                                                                                                                                    (wing_.M*(alpha - wing_.alpha0))))*(1 + exp((wing_.M*(alpha + wing_.alpha0)))));
            double CL_a = (1 - sigma_a)*(CL_.O + CL_.alpha*alpha) + sigma_a*(2.0*sign*pow(sin(alpha), 2.0)*cos(alpha));
            double AR = (pow(wing_.b, 2.0))/wing_.S;
            double CD_a = CD_.p + ((pow((CL_.O + CL_.alpha*(alpha)),
                                        2.0))/(3.14159*0.9 *
                                               AR)); //the const 0.9 in this equation replaces the e (Oswald Factor) variable and may be inaccurate

            double CX_a = -CD_a*cos(alpha) + CL_a*sin(alpha);
            double CX_q_a = -CD_.q*cos(alpha) + CL_.q*sin(alpha);
            double CX_deltaE_a = -CD_.delta_e*cos(alpha) + CL_.delta_e*sin(alpha);

            double CZ_a = -CD_a*sin(alpha) - CL_a*cos(alpha);
            double CZ_q_a = -CD_.q*sin(alpha) - CL_.q*cos(alpha);
            double CZ_deltaE_a = -CD_.delta_e*sin(alpha) - CL_.delta_e*cos(alpha);

            forces_.Fx = 0.5*(rho_)*pow(Va, 2.0)*wing_.S*(CX_a + (CX_q_a*wing_.c*q) / (2.0*Va) + CX_deltaE_a*delta_.e) + 0.5*rho_*prop_.S*prop_.C*(pow((prop_.k_motor*delta_.t), 2.0) - pow(Va, 2.0)) + fg[0];
            forces_.Fy = 0.5*(rho_)*pow(Va, 2.0)*wing_.S*(CY_.O + CY_.beta*beta + ((CY_.p*wing_.b*p) / (2.0*Va)) + ((CY_.r*wing_.b*r)/(2.0*Va)) + CY_.delta_a*delta_.a + CY_.delta_r*delta_.r) + fg[1];
            forces_.Fz = 0.5*(rho_)*pow(Va, 2.0)*wing_.S*(CZ_a + (CZ_q_a*wing_.c*q) / (2.0*Va) + CZ_deltaE_a*delta_.e) + fg[2];
            forces_.l = 0.5*(rho_)*pow(Va, 2.0)*wing_.S*wing_.b*(Cell_.O + Cell_.beta*beta + (Cell_.p*wing_.b*p) / (2.0*Va) + (Cell_.r*wing_.b*r)/(2.0*Va) + Cell_.delta_a*delta_.a + Cell_.delta_r*delta_.r) - prop_.k_T_P * pow((prop_.k_Omega*delta_.t), 2.0);
            forces_.m = 0.5*(rho_)*pow(Va, 2.0)*wing_.S*wing_.c*(Cm_.O + Cm_.alpha*alpha + (Cm_.q*wing_.c*q) / (2.0*Va) + Cm_.delta_e*delta_.e);
            forces_.n = 0.5*(rho_)*pow(Va, 2.0)*wing_.S*wing_.b*(Cn_.O + Cn_.beta*beta + (Cn_.p*wing_.b*p) / (2.0*Va) + (Cn_.r*wing_.b*r)/(2.0*Va) + Cn_.delta_a*delta_.a + Cn_.delta_r*delta_.r);
        }
        else
        {
            if (!std::isfinite(Va))
            {
                std::cout << "We had a Nan or Infinite" << std::endl;
            }
            else
            {
                forces_.Fx = 0.5*rho_*prop_.S*prop_.C*(pow((prop_.k_motor*delta_.t), 2.0));
                forces_.Fy = 0.0;
                forces_.Fz = 0.0;
                forces_.l = 0.0;
                forces_.m = 0.0;
                forces_.n = 0.0;
            }
        }
    }

    void AircraftForcesAndMoments::SendForces()
    {
        // Make sure we are applying reasonable forces
        if (std::isfinite(forces_.Fx + forces_.Fy + forces_.Fz + forces_.l + forces_.m + forces_.n))
        {
            geometry_msgs::msg::Wrench forces_moments_msg;

            forces_moments_msg.force.x = forces_.Fx;
            forces_moments_msg.force.y = forces_.Fy;
            forces_moments_msg.force.z = forces_.Fz;

            forces_moments_msg.torque.x = forces_.l;
            forces_moments_msg.torque.y = forces_.m;
            forces_moments_msg.torque.z = forces_.n;

            // Publish forces and moments.
            forces_moments_pub_->publish(forces_moments_msg);

        }
    }
}

int main(int argc, char * argv[])
{
    rclcpp:: init(argc, argv);
    rclcpp::spin(std::make_shared<rosplane2::AircraftForcesAndMoments>());
    rclcpp::shutdown();
    return 0;
}