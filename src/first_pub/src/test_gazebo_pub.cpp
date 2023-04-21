#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

using nav_msgs::msg::Odometry;


class FirstPublisher : public rclcpp::Node 
{
    public:
    FirstPublisher()
    : Node("test_gazebo_pub"), count_(0)
    {
        publisher_ = this->create_publisher<Odometry>("gazebo_truth", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&FirstPublisher::timer_callback, this));
    }

    private:

    void timer_callback()
    {
        auto message = Odometry();
        message.child_frame_id = "first Odometry";
        RCLCPP_INFO(this->get_logger(), message.child_frame_id.c_str());
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<Odometry>::SharedPtr publisher_;
    size_t count_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FirstPublisher>());
    rclcpp::shutdown();
    return 0;
}