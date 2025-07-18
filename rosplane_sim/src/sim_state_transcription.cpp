#include <memory>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rosplane_msgs/msg/state.hpp"
#include "rosflight_msgs/msg/sim_state.hpp"

#define RAD_2_DEG 180.0 / M_PI

using namespace std::chrono_literals;
using std::placeholders::_1;

class SimStateTranscription : public rclcpp::Node
{
public:
  SimStateTranscription()
      : Node("rosplane_state_transcription")
  {
    sim_state_subscription_ = this->create_subscription<rosflight_msgs::msg::SimState>(
      "sim/truth_state", 10, std::bind(&SimStateTranscription::publish_truth, this, _1));
    wind_truth_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "sim/wind_truth", 10, std::bind(&SimStateTranscription::wind_callback, this, _1));

    rosplane_state_publisher_ = this->create_publisher<rosplane_msgs::msg::State>("sim/rosplane/state", 10);
  }

private:
  rclcpp::Subscription<rosflight_msgs::msg::SimState>::SharedPtr sim_state_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr wind_truth_subscription_;

  rclcpp::Publisher<rosplane_msgs::msg::State>::SharedPtr rosplane_state_publisher_;

  double wn_ = 0.0;
  double we_ = 0.0;
  double wd_ = 0.0;

  void wind_callback(const geometry_msgs::msg::Vector3Stamped & msg) {
    // Wind velocities in the inertial frame
    wn_ = msg.vector.x;
    we_ = msg.vector.y;
    wd_ = msg.vector.z;
  }

  void publish_truth(const rosflight_msgs::msg::SimState & msg)
  {
    rosplane_msgs::msg::State state;

    // TODO: Should this use the same timestamp as the incoming message?
    state.header.stamp = this->get_clock()->now();
    state.header.frame_id = 1; // Denotes global frame.

    state.initial_lat = 0.0;
    state.initial_lon = 0.0; // TODO: implement correct initial lat and lon
    state.initial_alt = 0.0;

    // Inertial NED frame
    state.position[0] = msg.pose.position.x;
    state.position[1] = msg.pose.position.y;
    state.position[2] = msg.pose.position.z;

    // Quaternion is from body to inertial
    Eigen::Quaternionf q;
    q.w() = msg.pose.orientation.w;
    q.x() = msg.pose.orientation.x;
    q.y() = msg.pose.orientation.y;
    q.z() = msg.pose.orientation.z;

    // Equation B.1 in Small Unmanned Aircraft
    state.phi = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                     pow(q.w(), 2) + pow(q.z(), 2) - pow(q.x(), 2) - pow(q.y(), 2));
    state.theta = asin(2.0 * (q.w() * q.y() - q.x() * q.z()));
    state.psi = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                     pow(q.w(), 2) + pow(q.x(), 2) - pow(q.y(), 2) - pow(q.z(), 2));

    // Inertial linear velocities in the body frame
    double u = msg.twist.linear.x;
    double v = msg.twist.linear.y;
    double w = msg.twist.linear.z;

    state.u = u;
    state.v = v;
    state.w = w;

    // TODO: Should it be only the planar velocity?
    state.vg = std::sqrt(pow(u, 2) + pow(v, 2) + pow(w, 2));

    // Inertial angular velocities in the body frame
    state.p = msg.twist.angular.x;
    state.q = msg.twist.angular.y;
    state.r = msg.twist.angular.z;

    // Wind in the inertial frame
    state.wn = wn_;
    state.we = we_;

    // Components of the airspeed in the body frame
    Eigen::Vector3f v_i_b(u, v, w);
    Eigen::Vector3f v_w_i(wn_, we_, wd_);
    Eigen::Vector3f va = v_i_b - q.inverse() * v_w_i; // Rotate wind from inertial to body frame

    state.va = va.norm();

    Eigen::Vector3f v_i_i = q * v_i_b; // rotate body frame velocities into inertial frame
    state.chi = atan2(v_i_i(1), v_i_i(0));
    state.alpha = atan2(va(2), va(0));
    state.beta = asin(va(1) / state.va);

    state.quat_valid = true;

    state.quat[0] = msg.pose.orientation.w;
    state.quat[1] = msg.pose.orientation.x;
    state.quat[2] = msg.pose.orientation.y;
    state.quat[3] = msg.pose.orientation.z;

    state.chi_deg = RAD_2_DEG * state.chi;
    state.psi_deg = RAD_2_DEG * state.psi;

    rosplane_state_publisher_->publish(state);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimStateTranscription>());
  rclcpp::shutdown();
  return 0;
}
