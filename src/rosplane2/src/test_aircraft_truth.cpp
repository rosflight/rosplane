#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rosplane2_msgs/msg/state.hpp"

using std::placeholders::_1;
using nav_msgs::msg::Odometry;
using rosplane2_msgs::msg::State;

class AircraftTruth : public rclcpp::Node
{
    public:
        AircraftTruth()
        : Node("test_aircraft_truth")
        {
            subscription_ = this->create_subscription<Odometry>("gazebo_truth", 10, std::bind(&AircraftTruth::topic_callback, this, _1));
            publisher_ = this->create_publisher<State>("rosplane2_truth", 10);

        }

    private:
        rclcpp::Subscription<Odometry>::SharedPtr subscription_;
        rclcpp::Publisher<State>::SharedPtr publisher_;

        void topic_callback(const Odometry & msg) const
        {
            // get the info from the gazebo message
            auto gazeboPosition = msg.pose.pose.position;
            auto gazeboOrient = msg.pose.pose.orientation;

            // set up the state message object
            auto truthState = State();

            truthState.initial_lat = 0;
            truthState.initial_lon = 0;
            truthState.initial_alt = 0;

            truthState.position[0] = gazeboPosition.x;
            truthState.position[1] = gazeboPosition.y;
            truthState.position[2] = gazeboPosition.z;




            RCLCPP_INFO(this->get_logger(), "Sub heard: '%s'", msg.child_frame_id.c_str());
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