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

using std::placeholders::_1;
using nav_msgs::msg::Odometry;
using rosplane2_msgs::msg::State;

class AircraftTruth : public rclcpp::Node
{
    public:
        AircraftTruth()
        : Node("test_aircraft_truth")
        {
            subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("truth/odometry", 10, std::bind(&AircraftTruth::topic_callback, this, _1));
            publisher_ = this->create_publisher<rosplane2_msgs::msg::State>("state", 10);

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
        rclcpp::Subscription<Odometry>::SharedPtr subscription_;
        rclcpp::Publisher<State>::SharedPtr publisher_;

        int step = 0;

        bool debug = true;

        std::vector<std::vector<std::string>> state_values;

        void topic_callback(const Odometry & msg)
        {
            // get the info from the gazebo message
            auto gazeboPosition = msg.pose.pose.position;
            auto gazeboOrient = msg.pose.pose.orientation;
            auto gazeboLinVelocity = msg.twist.twist.linear;
            auto gazeboAngVelocity = msg.twist.twist.angular;

            // set up the state message object
            auto truthState = State();

            truthState.initial_lat = 0;
            truthState.initial_lon = 0;
            truthState.initial_alt = 0;

            truthState.position[0] = gazeboPosition.y;
            truthState.position[1] = gazeboPosition.x;
            truthState.position[2] = -gazeboPosition.z;

            truthState.quat[0]= gazeboOrient.w;
            truthState.quat[1]= gazeboOrient.x;
            truthState.quat[2]= gazeboOrient.y;
            truthState.quat[3]= gazeboOrient.z;
            truthState.quat_valid = true;

            truthState.theta = atan2(2*(gazeboOrient.w * gazeboOrient.x + gazeboOrient.y * gazeboOrient.z), pow(gazeboOrient.w,2.0) + pow(gazeboOrient.z,2.0) - pow(gazeboOrient.x,2.0) - pow(gazeboOrient.y,2.0));
            truthState.phi = asin(2*(gazeboOrient.w * gazeboOrient.y - gazeboOrient.x * gazeboOrient.z));
            truthState.psi = -atan2(2*(gazeboOrient.w * gazeboOrient.z + gazeboOrient.x * gazeboOrient.y), pow(gazeboOrient.w,2.0) + pow(gazeboOrient.x,2.0) - pow(gazeboOrient.y,2.0) - pow(gazeboOrient.z,2.0));

            float quat_array[4] = {truthState.quat[0], truthState.quat[1], truthState.quat[2], truthState.quat[3]};

            Eigen::Quaternionf quat = Eigen::Map<Eigen::Quaternionf>(quat_array);

            Eigen::Matrix3f rot = quat.toRotationMatrix();

            Eigen::Vector3f vel(gazeboLinVelocity.x, gazeboLinVelocity.y, gazeboLinVelocity.z);

            vel = rot.inverse()*vel;

            truthState.u = vel[1];
            truthState.v = vel[0];
            truthState.w = -vel[2];

            Eigen::Vector3f angle_vel(gazeboAngVelocity.x, gazeboAngVelocity.y, gazeboAngVelocity.z);

            angle_vel = rot.inverse()*angle_vel;

            truthState.p = angle_vel[1];
            truthState.q = angle_vel[0];
            truthState.r = -angle_vel[2];

            truthState.wn = 0; //not doing wind for now
            truthState.we = 0; 
            double wd = 0; 

            double ur = truthState.u - truthState.wn;
            double vr = truthState.v - truthState.we;
            double wr = truthState.w - wd;

            truthState.va = sqrt(pow(ur, 2.0) + pow(vr, 2.0) + pow(wr, 2.0));
            truthState.chi = atan2(truthState.va*sin(truthState.psi), truthState.va*cos(truthState.psi));
            truthState.alpha = atan2(wr, ur);
            truthState.beta = asin(vr/truthState.va);

            truthState.psi_deg = fmod(truthState.psi, 2.0*M_PI)*180.0 / M_PI; //-360 to 360
            truthState.psi_deg += (truthState.psi_deg < -180.0 ? 360.0 : 0.0);
            truthState.psi_deg -= (truthState.psi_deg > 180.0 ? 360.0 : 0.0);
            truthState.chi_deg = fmod(truthState.chi, 2.0*M_PI)*180.0 / M_PI; //-360 to 360
            truthState.chi_deg += (truthState.chi_deg < -180.0 ? 360.0 : 0.0);
            truthState.chi_deg -= (truthState.chi_deg > 180.0 ? 360.0 : 0.0);

            //RCLCPP_INFO(this->get_logger(), "Sub heard: '%s'", msg.child_frame_id.c_str());

            /// Debugging by high jacking the gazebo state and publishing from the csv.


            if (!debug) {
                publisher_->publish(truthState);
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


            publisher_->publish(truthState);

            step++;

        }


};

int main(int argc, char * argv[])
{
    rclcpp:: init(argc, argv);
    rclcpp::spin(std::make_shared<AircraftTruth>());
    rclcpp::shutdown();
    return 0;
}