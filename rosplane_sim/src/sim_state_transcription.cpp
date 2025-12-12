#include <chrono>
#include <future>
#include <memory>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <rosflight_msgs/srv/param_get.hpp>
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
    rclcpp::SubscriptionOptions options;
    cb_group_subs_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_clients_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    options.callback_group = cb_group_subs_;

    sim_state_subscription_ = this->create_subscription<rosflight_msgs::msg::SimState>(
      "sim/truth_state", 10, std::bind(&SimStateTranscription::publish_truth, this, _1), options);
    wind_truth_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "sim/truth_wind", 10, std::bind(&SimStateTranscription::wind_callback, this, _1), options);
    gyro_bias_truth_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "sim/truth_imu_bias", 10, std::bind(&SimStateTranscription::bias_callback, this, _1), options);

    rosplane_state_publisher_ = this->create_publisher<rosplane_msgs::msg::State>("sim/rosplane/state", 10);

    firmware_param_get_client_ = this->create_client<rosflight_msgs::srv::ParamGet>("param_get", rmw_qos_profile_default, cb_group_clients_);
  }

private:
  rclcpp::Subscription<rosflight_msgs::msg::SimState>::SharedPtr sim_state_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr wind_truth_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gyro_bias_truth_subscription_;

  rclcpp::Client<rosflight_msgs::srv::ParamGet>::SharedPtr firmware_param_get_client_;

  rclcpp::Publisher<rosplane_msgs::msg::State>::SharedPtr rosplane_state_publisher_;

  rclcpp::CallbackGroup::SharedPtr cb_group_subs_;
  rclcpp::CallbackGroup::SharedPtr cb_group_clients_;

  double wn_ = 0.0;
  double we_ = 0.0;
  double wd_ = 0.0;
  
  double bx_ = 0.0;
  double by_ = 0.0;
  double bz_ = 0.0;
  
  double firmware_bias_x_ = 0.0;
  double firmware_bias_y_ = 0.0;
  double firmware_bias_z_ = 0.0;
  
  double init_lat_ = 0.0;
  double init_lon_ = 0.0;
  double init_alt_ = 0.0;

  bool firmware_biases_set_ = false;
  bool init_lat_lon_alt_set_ = false;

  void wind_callback(const geometry_msgs::msg::Vector3Stamped & msg) {
    // Wind velocities in the inertial frame
    wn_ = msg.vector.x;
    we_ = msg.vector.y;
    wd_ = msg.vector.z;
  }
  
  void bias_callback(const sensor_msgs::msg::Imu & msg) {
    if (!firmware_biases_set_) {
      firmware_param_get_client_->wait_for_service(std::chrono::milliseconds(1000));
      auto req = std::make_shared<rosflight_msgs::srv::ParamGet::Request>();
      std::string param_name = "GYRO_X_BIAS";
      req->name = param_name;
      auto gyro_x_bias_future = firmware_param_get_client_->async_send_request(req);
      std::future_status status = gyro_x_bias_future.wait_for(std::chrono::milliseconds(1000)); // This times out!
      
      if (status == std::future_status::timeout) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Timed out.");
        return;
      }

      rosflight_msgs::srv::ParamGet::Response::SharedPtr response = gyro_x_bias_future.get();
      firmware_bias_x_ = response->value;

      req->name = "GYRO_Y_BIAS";
      auto gyro_y_bias_future = firmware_param_get_client_->async_send_request(req);
      status = gyro_y_bias_future.wait_for(std::chrono::milliseconds(1000));

      if (status == std::future_status::timeout) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Timed out.");
        return;
      }

      response = gyro_y_bias_future.get();
      firmware_bias_y_ = response->value;

      req->name = "GYRO_Z_BIAS";
      auto gyro_z_bias_future = firmware_param_get_client_->async_send_request(req);
      status = gyro_z_bias_future.wait_for(std::chrono::milliseconds(1000));

      if (status == std::future_status::timeout) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Timed out.");
        return;
      }

      response = gyro_z_bias_future.get();
      firmware_bias_z_ = response->value;

      firmware_biases_set_ = true;
    }

    bx_ = msg.angular_velocity.x - firmware_bias_x_;
    by_ = msg.angular_velocity.y - firmware_bias_y_;
    bz_ = msg.angular_velocity.z - firmware_bias_z_;
  }

  void publish_truth(const rosflight_msgs::msg::SimState & msg)
  {
    rosplane_msgs::msg::State state;

    state.header.stamp = msg.header.stamp;
    state.header.frame_id = 1; // Denotes global frame.
    
    if (!init_lat_lon_alt_set_) {

      auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(this, "/standalone_sensors");
      
      while (!parameters_client->wait_for_service(std::chrono::milliseconds(1000))) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
      }

      auto gnss_origin = parameters_client->get_parameters({"origin_altitude", "origin_latitude", "origin_longitude"}).get();

      init_alt_ = gnss_origin.at(0).as_double();
      init_lat_ = gnss_origin.at(1).as_double();
      init_lon_ = gnss_origin.at(2).as_double();

      init_lat_lon_alt_set_ = true;

    }

    state.initial_lat = init_lat_;
    state.initial_lon = init_lon_;
    state.initial_alt = init_alt_;

    // Inertial NED frame
    state.p_n = msg.pose.position.x;
    state.p_e = msg.pose.position.y;
    state.p_d = msg.pose.position.z;

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
    double v_x = msg.twist.linear.x;
    double v_y = msg.twist.linear.y;
    double v_z = msg.twist.linear.z;

    state.v_x = v_x;
    state.v_y = v_y;
    state.v_z = v_z;

    // Inertial angular velocities in the body frame
    state.p = msg.twist.angular.x;
    state.q = msg.twist.angular.y;
    state.r = msg.twist.angular.z;
    
    // Gyro biases
    state.b_x = bx_;
    state.b_y = by_;
    state.b_z = bz_;

    // Wind in the inertial frame
    state.wn = wn_;
    state.we = we_;

    // Components of the airspeed in the body frame
    Eigen::Vector3f v_i_b(v_x, v_y, v_z);
    Eigen::Vector3f v_w_i(wn_, we_, wd_);
    Eigen::Vector3f va = v_i_b - q.inverse() * v_w_i; // Rotate wind from inertial to body frame

    state.va = va.norm();

    Eigen::Vector3f v_i_i = q * v_i_b; // rotate body frame velocities into inertial frame
    state.chi = atan2(v_i_i(1), v_i_i(0));
    state.alpha = atan2(va(2), va(0));
    state.beta = asin(va(1) / state.va);

    state.quat.w = msg.pose.orientation.w;
    state.quat.x = msg.pose.orientation.x;
    state.quat.y = msg.pose.orientation.y;
    state.quat.z = msg.pose.orientation.z;

    rosplane_state_publisher_->publish(state);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node =  std::make_shared<SimStateTranscription>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
