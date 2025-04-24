
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include "rosplane_msgs/msg/state.hpp"
#include "rosflight_msgs/msg/sim_state.hpp"

using namespace std::chrono_literals;
using rosplane_msgs::msg::State;
using std::placeholders::_1;

class SimStateTranscription : public rclcpp::Node
{
public:
  SimStateTranscription()
      : Node("sim_state_transcriber")
  {
    sim_state_subscription = this->create_subscription<rosflight_msgs::msg::SimState>(
      "/sim/truth_state", 10, std::bind(&SimStateTranscription::publish_truth, this, _1));

    publisher_ = this->create_publisher<rosplane_msgs::msg::State>("/sim/fixedwing/state", 10);
  }

private:
  rclcpp::Subscription<rosflight_msgs::msg::SimState>::SharedPtr sim_state_subscription;

  rclcpp::Publisher<State>::SharedPtr publisher_;

  //TODO insert wind callback.
  double wn_ = 0.0;
  double we_ = 0.0;
  double wd_ = 0.0;

  void publish_truth(const rosflight_msgs::msg::SimState & msg)
  {

    rosplane_msgs::msg::State fixedwing_state;

    fixedwing_state.header.stamp = this->get_clock()->now();
    fixedwing_state.header.frame_id = 1; // Denotes global frame.

    fixedwing_state.initial_lat = 0.0;
    fixedwing_state.initial_lon = 0.0; // TODO implement correct initial lat and lon
    fixedwing_state.initial_alt = 0.0;

    fixedwing_state.position[0] = msg.pose.position.x;
    fixedwing_state.position[1] = msg.pose.position.y;
    fixedwing_state.position[2] = msg.pose.position.z;

    Eigen::Quaternionf q;
    q.w() = msg.pose.orientation.w;
    q.x() = msg.pose.orientation.x;
    q.y() = msg.pose.orientation.y;
    q.z() = msg.pose.orientation.z;

    Eigen::Vector3f euler;
    euler(0) = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                     pow(q.w(), 2) + pow(q.z(), 2) - pow(q.x(), 2) - pow(q.y(), 2));
    euler(1) = asin(2.0 * (q.w() * q.y() - q.x() * q.z()));
    euler(2) = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                     pow(q.w(), 2) + pow(q.x(), 2) - pow(q.y(), 2) - pow(q.z(), 2));

    fixedwing_state.phi = euler(0);
    fixedwing_state.theta = euler(1);
    fixedwing_state.psi = euler(2);

    double u = msg.twist.linear.x;
    double v = msg.twist.linear.y;
    double w = msg.twist.linear.z;

    fixedwing_state.u = u;
    fixedwing_state.v = v;
    fixedwing_state.w = w;

    fixedwing_state.vg = std::sqrt(pow(u, 2) + pow(v, 2) + pow(w, 2));

    fixedwing_state.p = msg.twist.angular.x;
    fixedwing_state.q = msg.twist.angular.y;
    fixedwing_state.r = msg.twist.angular.z;

    fixedwing_state.wn = wn_;
    fixedwing_state.we = we_;

    double ur = u - wn_;
    double vr = v - we_;
    double wr = w - wd_;

    fixedwing_state.va = std::sqrt(pow(ur, 2) + pow(vr, 2) + pow(wr, 2));

    fixedwing_state.chi = atan2(fixedwing_state.va * sin(fixedwing_state.psi), fixedwing_state.va * cos(fixedwing_state.psi));
    fixedwing_state.alpha = atan2(wr, ur);
    fixedwing_state.beta = asin(vr / fixedwing_state.va);

    fixedwing_state.quat_valid = true;

    fixedwing_state.quat[0] = msg.pose.orientation.w;
    fixedwing_state.quat[1] = msg.pose.orientation.x;
    fixedwing_state.quat[2] = msg.pose.orientation.y;
    fixedwing_state.quat[3] = msg.pose.orientation.z;

    //    fixedwing_state.psi_deg = fixedwing_state.psi * TODO implement the deg into the fixedwing_state.

    publisher_->publish(fixedwing_state);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimStateTranscription>());
  rclcpp::shutdown();
  return 0;
}
