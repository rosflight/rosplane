#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rosplane2_msgs/msg/state.hpp"
#include "Eigen/Geometry"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <geometry_msgs/msg/wrench.hpp>
#include "chrono"

using namespace std::chrono_literals;
using std::placeholders::_1;
using rosplane2_msgs::msg::State;

class AircraftTruth : public rclcpp::Node
{
    public:
        AircraftTruth()
        : Node("test_aircraft_truth")
        {
            subscription_ = this->create_subscription<geometry_msgs::msg::Wrench>("forces_moments", 10, std::bind(&AircraftTruth::forces_moments_callback, this, _1));
            publisher_ = this->create_publisher<rosplane2_msgs::msg::State>("state", 10);

            timer_ = this->create_wall_timer(10ms, std::bind(&AircraftTruth::update, this));

            prev_state[0] = 0.0;
            prev_state[1] = 0.0;
            prev_state[2] = 0.0;
            prev_state[3] = 30.0;
            prev_state[4] = 0.0;
            prev_state[5] = 0.0;
            prev_state[6] = 1.0;
            prev_state[7] = 0.0;
            prev_state[8] = 0.0;
            prev_state[9] = 0.0; // TODO find out why the controller is now freaking out (jumping between the hold, climb, descend, takeoff).
            prev_state[10] = 0.0;
            prev_state[11] = 0.0;
            prev_state[12] = 0.0;



            std::string filename = "/home/ian/Downloads/state_updated.csv";

            std::ifstream file(filename);

            if (!file.is_open())
            {
                std::cout << "Failed to open file: " << filename << std::endl;
                debug = false;
            }
            else{

                std::string line;
                // Skip header
                getline(file, line);

                while (getline(file, line)) {

                    std::vector<std::string> tokens;
                    std::stringstream ss(line);
                    std::string token;

                    while (getline(ss, token, ','))
                    {
                        tokens.push_back(token);
                    }

                    state_values.push_back(tokens);
                }


            }
        }

    private:
        rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr subscription_;
        rclcpp::Publisher<State>::SharedPtr publisher_;

        int step = 0;

        bool debug = false;

        std::vector<std::vector<std::string>> state_values;

        rclcpp::TimerBase::SharedPtr timer_;

        void forces_moments_callback(const geometry_msgs::msg::Wrench & msg){
            forces_moments = msg;
        }


        void update()
        {

            // set up the state message object.
            rosplane2_msgs::msg::State truthState = rosplane2_msgs::msg::State();

            auto k1 = derivatives(prev_state, forces_moments);

            auto k2 = derivatives(prev_state + ts/2.0 * k1, forces_moments);

            auto k3 = derivatives(prev_state + ts/2.0 * k2, forces_moments);

            auto k4 = derivatives(prev_state + ts/2.0 * k3, forces_moments);

            prev_state += ts/6.0 *(k1 + 2*k2 + 2*k3 + k4);

            // Normalize the quaternion.

            float e0 = prev_state[6];
            float e1 = prev_state[7];
            float e2 = prev_state[8];
            float e3 = prev_state[9];

            float normE = std::sqrt(pow(e0, 2) + pow(e1, 2) + pow(e2, 2) + pow(e3, 2));

            prev_state[6] = e0/normE;
            prev_state[7] = e1/normE;
            prev_state[8] = e2/normE;
            prev_state[9] = e3/normE;

            // Update velocity data.

            // TODO add proper wind.

            Eigen::Matrix<float, 6, 1> wind;

            wind << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

            update_velocity_data(wind);

            // Update other variables and prev_state.

//            RCLCPP_INFO(this->get_logger(), "True state about to be updated.");

            update_true_state();

//            RCLCPP_INFO(this->get_logger(), "True state updated.");


            //Publish the new state.
            /// Debugging by high jacking the gazebo state and publishing from the csv.

            auto sys_now = std::chrono::high_resolution_clock::now();

            rclcpp::Time now = this->get_clock()->now();
            uint64_t nanoseconds_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            uint64_t seconds_since_epoch = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            true_state.header.stamp.sec = seconds_since_epoch;
            true_state.header.stamp.nanosec = nanoseconds_since_epoch - seconds_since_epoch * 1000000000;

//            RCLCPP_INFO_STREAM(this->get_logger(), seconds_since_epoch << ":" << nanoseconds_since_epoch - seconds_since_epoch * 1000000000);


            if (!debug) {
                publisher_->publish(true_state);
                return;
            }

            truthState.position[0] = std::stof(state_values[step][4]);
            truthState.position[1] = std::stof(state_values[step][5]);
            truthState.position[2] = std::stof(state_values[step][6]);

            truthState.va = std::stof(state_values[step][7]);

            truthState.alpha = std::stof(state_values[step][8]);
            truthState.beta = std::stof(state_values[step][9]);

            truthState.phi = std::stof(state_values[step][10]);
            truthState.theta = std::stof(state_values[step][11]);
            truthState.psi = std::stof(state_values[step][12]);


            truthState.p = std::stof(state_values[step][14]);
            truthState.q = std::stof(state_values[step][15]);
            truthState.r = std::stof(state_values[step][16]);

            truthState.vg = std::stof(state_values[step][17]);
            truthState.wn = std::stof(state_values[step][18]);
            truthState.we = std::stof(state_values[step][19]);

            truthState.quat[0] = std::stof(state_values[step][20]);
            truthState.quat[1] = std::stof(state_values[step][21]);
            truthState.quat[2] = std::stof(state_values[step][22]);
            truthState.quat[3] = std::stof(state_values[step][23]);
            truthState.quat_valid = std::stoi(state_values[step][24]);

            truthState.chi_deg = std::stof(state_values[step][25]);
            truthState.psi_deg = std::stof(state_values[step][26]);

            truthState.initial_lat = std::stof(state_values[step][27]);
            truthState.initial_lon = std::stof(state_values[step][28]);
            truthState.initial_alt = std::stof(state_values[step][29]);

            truthState.chi = std::stof(state_values[step][13]);


//            publisher_->publish(truthState);

            step++;

        }

        Eigen::Matrix<float, 13, 1> derivatives(Eigen::Matrix<float, 13, 1> state, geometry_msgs::msg::Wrench forces_moments){

            Eigen::Matrix<float, 13, 1> xdot;

            // Extract states

            float u = state[3];
            float v = state[4];
            float w = state[5];
            Eigen::Quaternionf quat;
            quat.w() = state[6];
            quat.x() = state[7];
            quat.y() = state[8];
            quat.z() = state[9];
            float p = state[10];
            float q = state[11];
            float r = state[12];

            // Extract forces and moments

            float fx = forces_moments.force.x;
            float fy = forces_moments.force.y;
            float fz = forces_moments.force.z;
            float l = forces_moments.torque.x;
            float m = forces_moments.torque.y;
            float n = forces_moments.torque.z;

            // Position kinematics

            Eigen::Vector3f vels;
            vels << u, v, w;

//            RCLCPP_INFO_STREAM(this->get_logger(), "vels: u = " << u << " v = " << v << " w = " << w);
            auto pos_dot = quat.toRotationMatrix()*vels;
            float north_dot = pos_dot[0];
            float east_dot = pos_dot[1];
            float down_dot = pos_dot[2];

            // Position dynamics

            float u_dot = r*v - q*w + fx/mass_;
            float v_dot = p*w - r*u + fy/mass_;
            float w_dot = q*u - p*v + fz/mass_;

            // Rotational Kinematics

            float e0_dot = 0.5 * (-p*quat.x() - q*quat.y() - r*quat.z());
            float e1_dot = 0.5 * (p*quat.w() + r*quat.y() - q*quat.z());
            float e2_dot = 0.5 * (q*quat.w() - r*quat.x() + p*quat.z());
            float e3_dot = 0.5 * (r*quat.w() + q*quat.x() - p*quat.y());

            // Rotational Dynamics

            float gamma = Jx * Jz - pow(Jxz, 2);
            float gamma1 = (Jxz * (Jx - Jy + Jz)) / gamma;
            float gamma2 = (Jz * (Jz - Jy) + pow(Jxz,2)) / gamma;
            float gamma3 = Jz / gamma;
            float gamma4 = Jxz / gamma;
            float gamma5 = (Jz - Jx) / Jy;
            float gamma6 = Jxz / Jy;
            float gamma7 = ((Jx - Jy) * Jx + pow(Jxz,2)) / gamma;
            float gamma8 = Jx / gamma;


            float p_dot = gamma1*p*q - gamma2*q*r + gamma3*l + gamma4*n;
            float q_dot = gamma5*p*r - gamma6*(pow(p, 2) - pow(r,2)) + m/Jy;
            float r_dot = gamma7*p*q - gamma1*q*r + gamma4*l + gamma8*n;

            xdot << north_dot, east_dot, down_dot, u_dot, v_dot, w_dot, e0_dot, e1_dot, e2_dot, e3_dot, p_dot, q_dot, r_dot;

            return xdot;
        }

        void update_velocity_data(Eigen::Matrix<float, 6, 1> wind){

            Eigen::Matrix<float, 3, 1> steady_state;
            steady_state << wind[0], wind[1], wind[2];

            Eigen::Matrix<float, 3, 1> gust;
            gust << wind[3], wind[4], wind[5];

            Eigen::Quaternionf quaternion;
            quaternion.w() = prev_state[6];
            quaternion.x() = prev_state[7];
            quaternion.y() = prev_state[8];
            quaternion.z() = prev_state[9];

            Eigen::Matrix3f rot = quaternion.toRotationMatrix(); // TODO make sure this is resulting in the correct rotation to get the right alpha.

            Eigen::Matrix<float, 3, 1> wind_body_frame = rot.transpose() * steady_state;

            wind_body_frame += gust;

            this->wind = rot * wind_body_frame;

            Eigen::Matrix<float, 3, 1> vels;

            vels << prev_state[3], prev_state[4], prev_state[5];

            Eigen::Matrix<float, 3, 1> v_air = vels - wind_body_frame;
            float ur = v_air[0];
            float vr = v_air[1];
            float wr = v_air[2];

            Va = std::sqrt(pow(ur, 2) + pow(vr, 2) + pow(wr, 2));

            if (ur == 0){
                alpha = std::copysign(1, ur)*M_PI/2.0;
            }
            else{
                alpha = atan(wr/ur);
            }

            float tmp = std::sqrt(pow(ur, 2) + pow(wr, 2));

            if (tmp == 0){
                beta = std::copysign(1, vr) * M_PI/2.0;
            }
            else{
                beta = std::asin(vr/tmp);
            }


        }

        void update_true_state(){

            true_state.position[0] = prev_state[0];
            true_state.position[1] = prev_state[1];
            true_state.position[2] = prev_state[2];

            true_state.va = Va;
            true_state.alpha = alpha;
            true_state.beta = beta;

            Eigen::Quaternionf quaternion;
            quaternion.w() = prev_state[6];
            quaternion.x() = prev_state[7];
            quaternion.y() = prev_state[8];
            quaternion.z() = prev_state[9];

            // Euler angles.

            true_state.phi = atan2(2.0*(quaternion.w()*quaternion.x() + quaternion.y() * quaternion.z()), pow(quaternion.w(), 2) + pow(quaternion.z(), 2) - pow(quaternion.x(), 2) - pow(quaternion.y(), 2));
            true_state.theta = asin(2.0*(quaternion.w()*quaternion.y() - quaternion.x()*quaternion.z()));
            true_state.psi = atan2(2.0*(quaternion.w()*quaternion.z() + quaternion.x()*quaternion.y()), pow(quaternion.w(), 2) + pow(quaternion.x(), 2) - pow(quaternion.y(), 2) - pow(quaternion.z(), 2));

//            RCLCPP_INFO_STREAM(this->get_logger(), "theta: " << true_state.theta <<  "alpha: " << alpha);


            Eigen::Matrix<float, 3, 1> vels;

            vels << prev_state[3], prev_state[4], prev_state[5];

//            RCLCPP_INFO_STREAM(this->get_logger(), "vels: u = " << prev_state[3] << " v = " << prev_state[4] << " w = " << prev_state[5]);

            true_state.u = vels[0];
            true_state.v = vels[1];
            true_state.w = vels[2];

            Eigen::Matrix<float, 3, 1> pdot = quaternion.toRotationMatrix() * vels;

            true_state.vg = pdot.norm();

            true_state.chi = std::atan2(pdot[1], pdot[0]);

            true_state.p = prev_state[10];
            true_state.q = prev_state[11];
            true_state.r = prev_state[12];

            true_state.wn = wind[0];
            true_state.we = wind[1];

            true_state.quat_valid = true;

            true_state.quat[0] = quaternion.w();
            true_state.quat[1] = quaternion.x();
            true_state.quat[2] = quaternion.y();
            true_state.quat[3] = quaternion.z();

            true_state.chi_deg = true_state.chi * M_PI/180.;
            true_state.psi_deg = true_state.psi * M_PI/180.;

            true_state.initial_alt = 0.0;
            true_state.initial_lon = 0.0;
            true_state.initial_lat = 0.0;

        }

        Eigen::Matrix<float, 13, 1> prev_state;
        Eigen::Matrix<float, 3, 1> wind;

        rosplane2_msgs::msg::State true_state;

        float Va;
        float alpha;
        float beta;

        geometry_msgs::msg::Wrench forces_moments;

        float ts = .01;

        double mass_ = 11.;

        float Jx = 11.;
        float Jy = 1.135;
        float Jz = 1.759;
        float Jxz = .1204;



};

int main(int argc, char * argv[])
{
    rclcpp:: init(argc, argv);
    rclcpp::spin(std::make_shared<AircraftTruth>());
    rclcpp::shutdown();
    return 0;
}