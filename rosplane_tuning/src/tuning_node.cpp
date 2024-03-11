#include <memory>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <string>

#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosflight_msgs/msg/rc_raw.hpp" // Include the message type for /rc_raw topic

class TuningNode : public rclcpp::Node
{
public:
    TuningNode() : Node("tuning_node")
    {
        // Subscribe to the /rc_raw topic
        subscription_ = this->create_subscription<rosflight_msgs::msg::RCRaw>(
            "/rc_raw", 10, std::bind(&TuningNode::rcRawCallback, this, std::placeholders::_1));

        // Initialize the publisher
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("tuning_output", 10);
       
       // Initialize the service server
        service_ = this->create_service<std_srvs::srv::Trigger>(
            "time_difference_service", std::bind(&TuningNode::timeDifferenceCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void rcRawCallback(const rosflight_msgs::msg::RCRaw::SharedPtr msg)
    {
        // Check the values at specific indices
        int value1 = msg->values[1];
        int value2 = msg->values[2];
        int value3 = msg->values[3];
        int value5 = msg->values[5];

        // Check conditions
        bool condition1 = (value1 > 1650 || value1 < 1350);
        bool condition2 = (value2 > 1650 || value2 < 1350);
        bool condition3 = (value3 > 1650 || value3 < 1350);
        bool condition5 = (value5 == 1000);

        // Publish 1 if conditions are met, otherwise publish 0
        std_msgs::msg::Int32 output_msg;
        if (condition1 || condition2 || condition3 || condition5) {
            output_msg.data = 1;
        } else {
            output_msg.data = 0;
        }
        publisher_->publish(output_msg);
    }

    void timeDifferenceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        // Implement the time difference calculation logic here...
        // Calculate time difference
        
        response->message = "Response Received";
    
    }

    rclcpp::Subscription<rosflight_msgs::msg::RCRaw>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TuningNode>());
  rclcpp::shutdown();
  return 0;
}
