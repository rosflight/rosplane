#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <rosplane2_msgs/msg/state.hpp>
using std::placeholders::_1;

class vehicle_state_sub_node : public rclcpp::Node
{
  public:
    vehicle_state_sub_node()
    : Node("vehicle_state_sub_")
    {
      subscription_ = this->create_subscription<rosplane2_msgs::msg::State>(
      "state", 10, std::bind(&vehicle_state_sub_node::topic_callback, this, _1));
    }

  private:
    void topic_callback(const rosplane2_msgs::msg::State & msg) const
    {
      //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

    }
    rclcpp::Subscription<rosplane2_msgs::msg::State>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vehicle_state_sub_node>());
  rclcpp::shutdown();
  return 0;
}