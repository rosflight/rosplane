#include "rclcpp/rclcpp.hpp"
#include "rosplane_msgs/msg/waypoint.hpp"
#include "rclcpp/logging.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "rosplane_msgs/msg/waypoint.hpp"
#include "rosplane_msgs/msg/state.hpp"
#include "rosplane_msgs/msg/current_path.hpp"
// #include "geometry_msgs/msg/point.hpp"
// #include "std_msgs/msg/header.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>

using std::placeholders::_1;

namespace rosplane_gcs
{

class rviz_waypoint_publisher : public rclcpp::Node
{
public:
    rviz_waypoint_publisher();
    ~rviz_waypoint_publisher();

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_wp_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_mesh_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_aircraft_path_pub_;
    rclcpp::Subscription<rosplane_msgs::msg::Waypoint>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<rosplane_msgs::msg::State>::SharedPtr vehicle_state_sub_;
    rclcpp::Subscription<rosplane_msgs::msg::Waypoint>::SharedPtr target_wp_sub_;
    rclcpp::Subscription<rosplane_msgs::msg::CurrentPath>::SharedPtr current_path_sub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> aircraft_tf2_broadcaster_;

    void new_wp_callback(const rosplane_msgs::msg::Waypoint & wp);
    void fillet_callback(const rosplane_msgs::msg::CurrentPath & cp);    // I added this line
    void state_update_callback(const rosplane_msgs::msg::State & state);
    void target_wp_callback(const rosplane_msgs::msg::Waypoint & wp);
    void update_list();
    void update_markers();
    void update_mesh();
    void update_aircraft_history();

    void declare_parameters();
    bool set_parameters_callback(const std::vector<rclcpp::Parameter> & parameters);

    rosplane_msgs::msg::State vehicle_state_;

    // Persistent rviz markers
    visualization_msgs::msg::Marker marker_list_;
    visualization_msgs::msg::Marker line_list_;
    std::vector<geometry_msgs::msg::Point> marker_points_;
    visualization_msgs::msg::Marker target_wp_marker_;
    visualization_msgs::msg::Marker aircraft_;
    visualization_msgs::msg::Marker aircraft_history_;
    std::vector<geometry_msgs::msg::Point> aircraft_history_points_;

    int num_wps_;
    int i;
    bool do_pub_fillet;
};

rviz_waypoint_publisher::rviz_waypoint_publisher()
    : Node("rviz_waypoint_publisher") {

    rclcpp::QoS qos_transient_local_20_(20);
    qos_transient_local_20_.transient_local();

    // Publishers
    rviz_wp_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rviz/waypoint", qos_transient_local_20_);
    rviz_mesh_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rviz/mesh", 5);
    rviz_aircraft_path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rviz/mesh_path", 5);
    aircraft_tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribers
    waypoint_sub_ = this->create_subscription<rosplane_msgs::msg::Waypoint>("waypoint_path", qos_transient_local_20_,
            std::bind(&rviz_waypoint_publisher::new_wp_callback, this, _1));
    vehicle_state_sub_ = this->create_subscription<rosplane_msgs::msg::State>("estimated_state", 10,
            std::bind(&rviz_waypoint_publisher::state_update_callback, this, _1));
    target_wp_sub_ = this->create_subscription<rosplane_msgs::msg::Waypoint>("target_waypoint", qos_transient_local_20_,
            std::bind(&rviz_waypoint_publisher::target_wp_callback, this, _1));

    current_path_sub_ = this->create_subscription<rosplane_msgs::msg::CurrentPath>("current_path", 10,
            std::bind(&rviz_waypoint_publisher::fillet_callback, this, _1));


    // Initialize aircraft
    aircraft_.header.frame_id = "stl_frame";
    aircraft_.ns = "vehicle";
    aircraft_.id = 0;
    aircraft_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    aircraft_.mesh_resource = "package://rosplane_gcs/resource/skyhunter.dae";
    aircraft_.mesh_use_embedded_materials = false;
    aircraft_.action = visualization_msgs::msg::Marker::ADD;
    aircraft_.pose.position.x = 0.0;
    aircraft_.pose.position.y = 0.0;
    aircraft_.pose.position.z = 0.0;
    aircraft_.pose.orientation.x = 0.0;
    aircraft_.pose.orientation.y = 0.0;
    aircraft_.pose.orientation.z = 0.0;
    aircraft_.pose.orientation.w = 1.0;
    aircraft_.scale.x = 5.0;
    aircraft_.scale.y = 5.0;
    aircraft_.scale.z = 5.0;
    aircraft_.color.r = 0.67f;
    aircraft_.color.g = 0.67f;
    aircraft_.color.b = 0.67f;
    aircraft_.color.a = 1.0;

    num_wps_ = 0;
    i = 0;

    declare_parameters();
    do_pub_fillet = true;
}

rviz_waypoint_publisher::~rviz_waypoint_publisher() {}

void rviz_waypoint_publisher::declare_parameters() {
    this->declare_parameter("scale", 5.0);
    this->declare_parameter("text_scale", 15.0);
    this->declare_parameter("arrow_scale", 1.0);
    this->declare_parameter("line_scale", 3.0);
    this->declare_parameter("path_pub_rate", 10);
    this->declare_parameter("max_path_hist", 10000);
}

void rviz_waypoint_publisher::new_wp_callback(const rosplane_msgs::msg::Waypoint & wp) {
    visualization_msgs::msg::Marker new_marker;

    if (wp.clear_wp_list) {

        rclcpp::Time now = this->get_clock()->now();
        // Publish one for each ns
        new_marker.header.stamp = now;
        new_marker.header.frame_id = "NED";
        new_marker.ns = "wp";
        new_marker.id = 0;
        new_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        rviz_wp_pub_->publish(new_marker);
        new_marker.ns = "text";
        rviz_wp_pub_->publish(new_marker);
        new_marker.ns = "wp_path";
        rviz_wp_pub_->publish(new_marker);

        // Clear line list
        marker_points_.clear();

        num_wps_ = 0;
        return;
    }
    
    // Add point to line and marker lists

    // Create marker
    rclcpp::Time now = this->get_clock()->now();
    new_marker.header.stamp = now;
    new_marker.header.frame_id = "NED";
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
    new_marker.color.r = 1.0f;
    new_marker.color.g = 0.0f;
    new_marker.color.b = 0.0f;
    new_marker.color.a = 1.0;

    // Add point to line list
    geometry_msgs::msg::Point new_p;
    new_p.x = wp.w[0];
    new_p.y = wp.w[1];
    new_p.z = wp.w[2];
    marker_points_.push_back(new_p);
    update_list();
    update_markers();

    // Add Text label to marker
    double text_scale = this->get_parameter("text_scale").as_double();
    double scale = this->get_parameter("scale").as_double();

    visualization_msgs::msg::Marker new_text;
    rclcpp::Time now = this->get_clock()->now();
    new_text.header.stamp = now;
    new_text.header.frame_id = "NED";
    new_text.ns = "text";
    new_text.id = num_wps_;
    new_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    new_text.action = visualization_msgs::msg::Marker::ADD;
    new_text.pose.position.x = wp.w[0];
    new_text.pose.position.y = wp.w[1];
    new_text.pose.position.z = wp.w[2] - scale - 1.0;
    new_text.scale.z = text_scale;
    new_text.color.r = 0.0f;
    new_text.color.g = 0.0f;
    new_text.color.b = 0.0f;
    new_text.color.a = 1.0;
    new_text.text = std::to_string(num_wps_);

    rviz_wp_pub_->publish(marker_list_);
    rviz_wp_pub_->publish(line_list_);
    rviz_wp_pub_->publish(new_text);

    ++num_wps_;
}

void rviz_waypoint_publisher::update_list() {
    double line_scale = this->get_parameter("line_scale").as_double();

void rviz_waypoint_publisher::fillet_callback(const rosplane_msgs::msg::CurrentPath & cp) {
    visualization_msgs::msg::Marker new_marker;
    visualization_msgs::msg::Marker fillet_line_list;
    std::vector<geometry_msgs::msg::Point> fillet_line_points;

    static double center_x;
    static double center_y;

    // Show fillet if path type is orbital
    if (cp.path_type == 0) {
        if (do_pub_fillet | (center_x != cp.c[0])) {
            center_x = cp.c[0];
            center_y = cp.c[1];
            double radius = cp.rho;
            int degrees = 360;
            double rotation = 2 * M_PI / degrees;

            for(int degree = 0; degree <= degrees; ++degree) {
                double angle_rotation = degree * rotation;
                double point_x = center_x + radius * cos(angle_rotation);
                double point_y = center_y + radius * sin(angle_rotation);

                // Add point to line list
                geometry_msgs::msg::Point new_p;
                new_p.x = point_x;
                new_p.y = point_y;
                new_p.z = cp.c[2];
                fillet_line_points.push_back(new_p);
                }
            rclcpp::Time now = this->get_clock()->now();
            fillet_line_list.header.stamp = now;
            fillet_line_list.header.frame_id = "NED";
            fillet_line_list.ns = "fillet";
            fillet_line_list.id = 0;
            fillet_line_list.type = visualization_msgs::msg::Marker::LINE_STRIP;
            fillet_line_list.action = visualization_msgs::msg::Marker::ADD;
            fillet_line_list.scale.x = 3.0;
            fillet_line_list.color.r = 1.0f;
            fillet_line_list.color.g = 0.443f;
            fillet_line_list.color.b = 0.0;
            fillet_line_list.color.a = 1.0;
            fillet_line_list.points = fillet_line_points;

            // Add Text label to fillet_line_list marker
            visualization_msgs::msg::Marker fillet_text;
            fillet_text.header.stamp = now;
            fillet_text.header.frame_id = "NED";
            fillet_text.ns = "fillet";
            fillet_text.id = 1;
            fillet_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            fillet_text.action = visualization_msgs::msg::Marker::ADD;
            fillet_text.pose.position.x = cp.c[0];
            fillet_text.pose.position.y = cp.c[1];
            fillet_text.pose.position.z = cp.c[2];
            fillet_text.scale.z = TEXT_SCALE;
            fillet_text.color.r = 1.0f;
            fillet_text.color.g = 0.443f;
            fillet_text.color.b = 0.0f;
            fillet_text.color.a = 1.0;
            fillet_text.text = "Current  Orbit";

            rviz_wp_pub_->publish(fillet_line_list);
            rviz_wp_pub_->publish(fillet_text);
            do_pub_fillet = false;
        }
    }

    //erase the circle and set do_pub_fillet == true
    if (cp.path_type == 1) {
        if (!do_pub_fillet) {
            rclcpp::Time now = this->get_clock()->now();
            fillet_line_list.header.stamp = now;
            fillet_line_list.header.frame_id = "NED";
            fillet_line_list.ns = "fillet";
            fillet_line_list.id = 0;
            fillet_line_list.action = visualization_msgs::msg::Marker::DELETEALL;
            rviz_wp_pub_->publish(fillet_line_list);

            visualization_msgs::msg::Marker fillet_text;
            fillet_text.header.stamp = now;
            fillet_text.header.frame_id = "NED";
            fillet_text.ns = "fillet";
            fillet_text.id = 1;
            fillet_text.action = visualization_msgs::msg::Marker::DELETEALL;
            rviz_wp_pub_->publish(fillet_text);

            // Clear line list
            fillet_line_points.clear();

            do_pub_fillet = true;
            return;
            }
    }
}

void rviz_waypoint_publisher::update_list() {   
    rclcpp::Time now = this->get_clock()->now();
    line_list_.header.stamp = now;
    line_list_.header.frame_id = "NED";
    line_list_.ns = "wp_path";
    line_list_.id = 0;
    line_list_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_list_.action = visualization_msgs::msg::Marker::ADD;
    line_list_.scale.x = line_scale;
    line_list_.color.r = 0.0f;
    line_list_.color.g = 1.0f;
    line_list_.color.b = 0.0f;
    line_list_.color.a = 1.0;
    line_list_.points = marker_points_;
}

void rviz_waypoint_publisher::update_markers() {
    double scale = this->get_parameter("scale").as_double();

    // Update marker array
    rclcpp::Time now = this->get_clock()->now();
    marker_list_.header.stamp = now;
    marker_list_.header.frame_id = "NED";
    marker_list_.ns = "wp";
    marker_list_.id = 0;
    marker_list_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker_list_.action = visualization_msgs::msg::Marker::ADD;
    marker_list_.scale.x = scale;
    marker_list_.scale.y = scale;
    marker_list_.scale.z = scale;
    marker_list_.color.r = 1.0f;
    marker_list_.color.g = 0.0f;
    marker_list_.color.b = 0.0f;
    marker_list_.color.a = 1.0;
    marker_list_.points = marker_points_;
}

void rviz_waypoint_publisher::update_aircraft_history() {
    rclcpp::Time now = this->get_clock()->now();
    aircraft_history_.header.stamp = now;
    aircraft_history_.header.frame_id = "NED";
    aircraft_history_.ns = "vehicle_path";
    aircraft_history_.id = 0;
    aircraft_history_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    aircraft_history_.action = visualization_msgs::msg::Marker::ADD;
    aircraft_history_.scale.x = 1.0;
    aircraft_history_.scale.y = 1.0;
    aircraft_history_.scale.z = 1.0;
    aircraft_history_.color.r = 0.0f;
    aircraft_history_.color.g = 0.0f;
    aircraft_history_.color.b = 0.0f;
    aircraft_history_.color.a = 1.0;
    aircraft_history_.points = aircraft_history_points_;

    int max_path_history = this->get_parameter("max_path_hist").as_int();

    // Restrict length of history
    int path_overflow = (int) aircraft_history_points_.size() - max_path_history;

    if (path_overflow > 0) {
        aircraft_history_points_.erase(aircraft_history_points_.begin(), aircraft_history_points_.begin() + path_overflow);
    }
}

void rviz_waypoint_publisher::update_mesh() {
    rclcpp::Time now = this->get_clock()->now();
    aircraft_.header.stamp = now;

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "NED";
    t.child_frame_id = "aircraft_body";
    t.transform.translation.x = vehicle_state_.position[0];
    t.transform.translation.y = vehicle_state_.position[1];
    t.transform.translation.z = vehicle_state_.position[2];

    tf2::Quaternion q;
    q.setRPY(vehicle_state_.phi, vehicle_state_.theta, vehicle_state_.chi);
    t.transform.rotation.x = q.x(); //0.0; //vehicle_state_.quat[0];
    t.transform.rotation.y = q.y(); //0.0; //vehicle_state_.quat[1];
    t.transform.rotation.z = q.z(); //0.0; //vehicle_state_.quat[2];
    t.transform.rotation.w = q.w(); //1.0; //vehicle_state_.quat[3];

    int path_pub_rate = this->get_parameter("path_pub_rate").as_int();

    // Update aircraft history
    if (i % path_pub_rate == 0) {
        geometry_msgs::msg::Point new_p;
        new_p.x = vehicle_state_.position[0];
        new_p.y = vehicle_state_.position[1];
        new_p.z = vehicle_state_.position[2];
        aircraft_history_points_.push_back(new_p);
        update_aircraft_history();

        rviz_aircraft_path_pub_->publish(aircraft_history_);
    }

    aircraft_tf2_broadcaster_->sendTransform(t);
    rviz_mesh_pub_->publish(aircraft_);
}

void rviz_waypoint_publisher::state_update_callback(const rosplane_msgs::msg::State & msg) {
    vehicle_state_ = msg;
    update_mesh();
}

void rviz_waypoint_publisher::target_wp_callback(const rosplane_msgs::msg::Waypoint & msg) {
    double scale = this->get_parameter("scale").as_double();
    double arrow_scale = this->get_parameter("arrow_scale").as_double();

    target_wp_marker_.header.frame_id = "NED";
    target_wp_marker_.header.stamp = this->get_clock()->now();
    target_wp_marker_.ns = "wp";
    target_wp_marker_.id = -1;
    target_wp_marker_.type = visualization_msgs::msg::Marker::ARROW;
    target_wp_marker_.action = visualization_msgs::msg::Marker::ADD;
    target_wp_marker_.scale.x = 2.0 * arrow_scale;
    target_wp_marker_.scale.y = 4.0 * arrow_scale;
    target_wp_marker_.scale.z = 3.0 * arrow_scale;
    target_wp_marker_.color.r = 0.0f;
    target_wp_marker_.color.g = 0.0f;
    target_wp_marker_.color.b = 1.0f;
    target_wp_marker_.color.a = 0.5;

    geometry_msgs::msg::Point pt1;
    geometry_msgs::msg::Point pt2;
    pt2.x = msg.w[0] - scale;
    pt2.y = msg.w[1] - scale;
    pt2.z = msg.w[2] - scale;
    pt1.x = pt2.x - 10.0 * arrow_scale;
    pt1.y = pt2.y - 10.0 * arrow_scale;
    pt1.z = pt2.z - 10.0 * arrow_scale;

    target_wp_marker_.points = {pt1, pt2};

    rviz_wp_pub_->publish(target_wp_marker_);
}

} // namespace rosplane_gcs

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rosplane_gcs::rviz_waypoint_publisher>();

    rclcpp::spin(node);

    return 0;
}
