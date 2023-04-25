#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rosplane2_msgs/msg/state.hpp"
#include "Eigen/Geometry"

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

        }

    private:
        rclcpp::Subscription<Odometry>::SharedPtr subscription_;
        rclcpp::Publisher<State>::SharedPtr publisher_;

        void topic_callback(const Odometry & msg) const
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
            publisher_->publish(truthState);
        }


};

int main(int argc, char * argv[])
{
    rclcpp:: init(argc, argv);
    rclcpp::spin(std::make_shared<AircraftTruth>());
    rclcpp::shutdown();
    return 0;
}