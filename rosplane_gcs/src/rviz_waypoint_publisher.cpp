#include "rclcpp/rclcpp.hpp"
#include "rosplane_msgs/msg/waypoint.hpp"
#include "rclcpp/logging.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "rosplane_msgs/msg/waypoint.hpp"
// #include "geometry_msgs/msg/pose.hpp"
// #include "std_msgs/msg/header.hpp"

#define SCALE 5.0
using std::placeholders::_1;

namespace rosplane_gcs
{

class rviz_waypoint_publisher : public rclcpp::Node
{
public:
    rviz_waypoint_publisher();
    ~rviz_waypoint_publisher();

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_waypoint_pub_;
    rclcpp::Subscription<rosplane_msgs::msg::Waypoint>::SharedPtr waypoint_sub_;

    void new_wp_callback(const rosplane_msgs::msg::Waypoint & wp);

    int num_wps_;
};

rviz_waypoint_publisher::rviz_waypoint_publisher()
    : Node("rviz_waypoint_publisher") {
    
    rviz_waypoint_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rviz/waypoint", 10);
    waypoint_sub_ = this->create_subscription<rosplane_msgs::msg::Waypoint>("waypoint_path", 10,
            std::bind(&rviz_waypoint_publisher::new_wp_callback, this, _1));
    
    num_wps_ = 0;
}

rviz_waypoint_publisher::~rviz_waypoint_publisher() {}

void rviz_waypoint_publisher::new_wp_callback(const rosplane_msgs::msg::Waypoint & wp) {
    visualization_msgs::msg::Marker new_marker;
    new_marker.header.frame_id = "NED";
    
    rclcpp::Time now = this->get_clock()->now();
    new_marker.header.stamp = now;
    new_marker.ns = "wp";
    new_marker.id = num_wps_;
    new_marker.type = visualization_msgs::msg::Marker::SPHERE;
    new_marker.action = visualization_msgs::msg::Marker::ADD;
    new_marker.pose.position.x = wp.w[0];
    new_marker.pose.position.y = wp.w[1];
    new_marker.pose.position.z = wp.w[2];
    new_marker.scale.x = SCALE;
    new_marker.scale.y = SCALE;
    new_marker.scale.z = SCALE;
    if (num_wps_ % 3 == 0) {
        new_marker.type = visualization_msgs::msg::Marker::CUBE;
        new_marker.color.r = 1.0f;
        new_marker.color.g = 0.0f;
        new_marker.color.b = 0.0f;
    }
    else {
        new_marker.color.r = 0.0f;
        new_marker.color.g = 1.0f;
        new_marker.color.b = 0.0f;
    }
    new_marker.color.a = 1.0;
    // new_marker.lifetime = rclcpp::Duration::Duration(0, 0);

    rviz_waypoint_pub_->publish(new_marker);

    ++num_wps_;
}

} // namespace rosplane_gcs

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rosplane_gcs::rviz_waypoint_publisher>();

    rclcpp::spin(node);

    return 0;
}