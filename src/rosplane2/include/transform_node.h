
#ifndef BUILD_TRANSFORM_NODE_H
#define BUILD_TRANSFORM_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <rosplane2_msgs/msg/state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
//#include <tf2_ros>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>

using std::placeholders::_1;

namespace rosplane2
{
class transform_node : public rclcpp::Node
{
public:
    transform_node();

private:
    rclcpp::Subscription<rosplane2_msgs::msg::State>::SharedPtr vehicle_state_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster;

    void state_callback(rosplane2_msgs::msg::State::SharedPtr msg);

    void publish_transform(rclcpp::Time stamp, std::string header_frame, std::string child_frame, Eigen::Matrix3f rot, std::vector<float> translation);

};
}




#endif //BUILD_TRANSFORM_NODE_H
