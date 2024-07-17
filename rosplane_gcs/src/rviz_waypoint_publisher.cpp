#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include "rosplane_msgs/msg/state.hpp"
#include "rosplane_msgs/msg/waypoint.hpp"

#define SCALE 5.0
#define TEXT_SCALE 15.0
#define PATH_PUBLISH_MOD 10
#define MAX_PATH_HISTORY 10000
using std::placeholders::_1;

namespace rosplane_gcs
{

class RvizWaypointPublisher : public rclcpp::Node
{
public:
  RvizWaypointPublisher();
  ~RvizWaypointPublisher();

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_wp_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_mesh_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_aircraft_path_pub_;
  rclcpp::Subscription<rosplane_msgs::msg::Waypoint>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<rosplane_msgs::msg::State>::SharedPtr vehicle_state_sub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> aircraft_tf2_broadcaster_;

  void new_wp_callback(const rosplane_msgs::msg::Waypoint & wp);
  void state_update_callback(const rosplane_msgs::msg::State & state);
  void update_list();
  void update_mesh();
  void update_aircraft_history();

  rosplane_msgs::msg::State vehicle_state_;

  // Persistent rviz markers
  visualization_msgs::msg::Marker line_list_;
  std::vector<geometry_msgs::msg::Point> line_points_;
  visualization_msgs::msg::Marker aircraft_;
  visualization_msgs::msg::Marker aircraft_history_;
  std::vector<geometry_msgs::msg::Point> aircraft_history_points_;

  int num_wps_;
  int i_;
};

RvizWaypointPublisher::RvizWaypointPublisher()
    : Node("rviz_waypoint_publisher")
{

  rclcpp::QoS qos_transient_local_20_(20);
  qos_transient_local_20_.transient_local();
  rviz_wp_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rviz/waypoint",
                                                                         qos_transient_local_20_);
  rviz_mesh_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rviz/mesh", 5);
  rviz_aircraft_path_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("rviz/mesh_path", 5);
  waypoint_sub_ = this->create_subscription<rosplane_msgs::msg::Waypoint>(
    "waypoint_path", qos_transient_local_20_,
    std::bind(&RvizWaypointPublisher::new_wp_callback, this, _1));
  vehicle_state_sub_ = this->create_subscription<rosplane_msgs::msg::State>(
    "estimated_state", 10, std::bind(&RvizWaypointPublisher::state_update_callback, this, _1));

  aircraft_tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

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
  i_ = 0;
}

RvizWaypointPublisher::~RvizWaypointPublisher() {}

void RvizWaypointPublisher::new_wp_callback(const rosplane_msgs::msg::Waypoint & wp)
{
  visualization_msgs::msg::Marker new_marker;

  RCLCPP_INFO_STREAM(this->get_logger(), wp.lla);

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
    line_points_.clear();

    num_wps_ = 0;
    return;
  }

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
  line_points_.push_back(new_p);
  update_list();

  // Add Text label to marker
  visualization_msgs::msg::Marker new_text;
  new_text.header.stamp = now;
  new_text.header.frame_id = "NED";
  new_text.ns = "text";
  new_text.id = num_wps_;
  new_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  new_text.action = visualization_msgs::msg::Marker::ADD;
  new_text.pose.position.x = wp.w[0];
  new_text.pose.position.y = wp.w[1];
  new_text.pose.position.z = wp.w[2] - SCALE - 1.0;
  new_text.scale.z = TEXT_SCALE;
  new_text.color.r = 0.0f;
  new_text.color.g = 0.0f;
  new_text.color.b = 0.0f;
  new_text.color.a = 1.0;
  new_text.text = std::to_string(num_wps_);

  rviz_wp_pub_->publish(new_marker);
  rviz_wp_pub_->publish(line_list_);
  rviz_wp_pub_->publish(new_text);

  ++num_wps_;
}

void RvizWaypointPublisher::update_list()
{
  rclcpp::Time now = this->get_clock()->now();
  line_list_.header.stamp = now;
  line_list_.header.frame_id = "NED";
  line_list_.ns = "wp_path";
  line_list_.id = 0;
  line_list_.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_list_.action = visualization_msgs::msg::Marker::ADD;
  line_list_.scale.x = 3.0;
  line_list_.color.r = 0.0f;
  line_list_.color.g = 1.0f;
  line_list_.color.b = 0.0f;
  line_list_.color.a = 1.0;
  line_list_.points = line_points_;
}

void RvizWaypointPublisher::update_aircraft_history()
{
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

  // Restrict length of history
  if (aircraft_history_points_.size() > MAX_PATH_HISTORY) {
    aircraft_history_points_.erase(aircraft_history_points_.begin());
  }
}

void RvizWaypointPublisher::update_mesh()
{
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

  // Update aircraft history
  if (i_ % PATH_PUBLISH_MOD == 0) {
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

void RvizWaypointPublisher::state_update_callback(const rosplane_msgs::msg::State & msg)
{
  vehicle_state_ = msg;
  update_mesh();
}

} // namespace rosplane_gcs

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rosplane_gcs::RvizWaypointPublisher>();

  rclcpp::spin(node);

  return 0;
}