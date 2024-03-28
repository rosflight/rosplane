#include "estimator_base.hpp"
#include "estimator_example.hpp"
//#include <sensor_msgs/nav_sat_status.hpp>

namespace rosplane
{

estimator_base::estimator_base()
    : Node("estimator_base"), params(this)
{

  vehicle_state_pub_ = this->create_publisher<rosplane_msgs::msg::State>("estimated_state", 10);

  gnss_fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    gnss_fix_topic_, 10, std::bind(&estimator_base::gnssFixCallback, this, std::placeholders::_1));
  gnss_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    gnss_vel_topic_, 10, std::bind(&estimator_base::gnssVelCallback, this, std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, 10, std::bind(&estimator_base::imuCallback, this, std::placeholders::_1));
  baro_sub_ = this->create_subscription<rosflight_msgs::msg::Barometer>(
    baro_topic_, 10, std::bind(&estimator_base::baroAltCallback, this, std::placeholders::_1));
  airspeed_sub_ = this->create_subscription<rosflight_msgs::msg::Airspeed>(
    airspeed_topic_, 10, std::bind(&estimator_base::airspeedCallback, this, std::placeholders::_1));
  status_sub_ = this->create_subscription<rosflight_msgs::msg::Status>(
    status_topic_, 10, std::bind(&estimator_base::statusCallback, this, std::placeholders::_1));

  update_timer_ = this->create_wall_timer(10ms, std::bind(&estimator_base::update, this));

  init_static_ = 0;
  baro_count_ = 0;
  armed_first_time_ = false;

  // Declare and set parameters with the ROS2 system
  declare_parameters();
  params.set_parameters();
}

void estimator_base::declare_parameters()
{
  params.declare_param("rho", 1.225);
  params.declare_param("gravity", 9.8);
}

// TODO add param callback.

void estimator_base::update()
{
  struct output_s output;

  if (armed_first_time_) {
    estimate(input_, output);
  } else {
    output.pn = output.pe = output.h = 0;
    output.phi = output.theta = output.psi = 0;
    output.alpha = output.beta = output.chi = 0;
    output.p = output.q = output.r = 0;
    output.Va = 0;
  }

  input_.gps_new = false;

  rosplane_msgs::msg::State msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id =
    1; // Denotes global frame TODO make sure all messages throughout ROSPlane have this

  msg.position[0] = -output.pn;
  msg.position[1] = -output.pe; // TODO find out why there are these minuses.
  msg.position[2] = -output.h;
  if (gps_init_) {
    msg.initial_lat = init_lat_;
    msg.initial_lon = init_lon_;
    msg.initial_alt = init_alt_;
  }
  msg.va = output.Va;
  msg.alpha = output.alpha;
  msg.beta = output.beta;
  msg.phi = output.phi;
  msg.theta = output.theta;
  msg.psi = output.psi;
  msg.chi = output.chi;
  msg.p = output.p;
  msg.q = output.q;
  msg.r = output.r;
  msg.vg = output.Vg;
  msg.wn = output.wn;
  msg.we = output.we;
  msg.quat_valid = true;

  Eigen::Quaternionf q;
  q = Eigen::AngleAxisf(output.phi, Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(output.theta, Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(output.psi, Eigen::Vector3f::UnitZ());

  msg.quat[0] = q.w();
  msg.quat[1] = q.x();
  msg.quat[2] = q.y();
  msg.quat[3] = q.z();

  msg.u = output.Va * cos(output.theta);
  msg.v = 0;
  msg.w = output.Va * sin(output.theta);

  msg.psi_deg = fmod(output.psi, 2.0 * M_PI) * 180 / M_PI; //-360 to 360
  msg.psi_deg += (msg.psi_deg < -180 ? 360 : 0);
  msg.psi_deg -= (msg.psi_deg > 180 ? 360 : 0);
  msg.chi_deg = fmod(output.chi, 2.0 * M_PI) * 180 / M_PI; //-360 to 360
  msg.chi_deg += (msg.chi_deg < -180 ? 360 : 0);
  msg.chi_deg -= (msg.chi_deg > 180 ? 360 : 0);

  vehicle_state_pub_->publish(msg);
}

void estimator_base::gnssFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  bool has_fix = msg->status.status
    >= sensor_msgs::msg::NavSatStatus::STATUS_FIX; // Higher values refer to augmented fixes
  if (!has_fix || !std::isfinite(msg->latitude)) {
    input_.gps_new = false;
    return;
  }
  if (!gps_init_ && has_fix) {
    RCLCPP_INFO_STREAM(this->get_logger(), "init_lat: " << msg->latitude);
    gps_init_ = true;
    init_alt_ = msg->altitude;
    init_lat_ = msg->latitude;
    init_lon_ = msg->longitude;
  } else {
    input_.gps_n = EARTH_RADIUS * (msg->latitude - init_lat_) * M_PI / 180.0;
    input_.gps_e =
      EARTH_RADIUS * cos(init_lat_ * M_PI / 180.0) * (msg->longitude - init_lon_) * M_PI / 180.0;
    input_.gps_h = msg->altitude - init_alt_;
    input_.gps_new = true;

    //      RCLCPP_INFO_STREAM(this->get_logger(), "gps_n: " << input_.gps_n);
  }
}

void estimator_base::gnssVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  double v_n = msg->twist.linear.x;
  double v_e = msg->twist.linear.y;
  //  double v_d = msg->twist.linear.z; // This variable was unused.
  double ground_speed = sqrt(v_n * v_n + v_e * v_e);
  double course =
    atan2(v_e, v_n); //Does this need to be in a specific range? All uses seem to accept anything.
  input_.gps_Vg = ground_speed;
  if (ground_speed > 0.3) //this is a magic number. What is it determined from?
    input_.gps_course = course;
}

void estimator_base::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  input_.accel_x = msg->linear_acceleration.x;
  input_.accel_y = msg->linear_acceleration.y;
  input_.accel_z = msg->linear_acceleration.z;

  input_.gyro_x = msg->angular_velocity.x;
  input_.gyro_y = msg->angular_velocity.y;
  input_.gyro_z = msg->angular_velocity.z;
}

void estimator_base::baroAltCallback(const rosflight_msgs::msg::Barometer::SharedPtr msg)
{
  // For readability, declare the parameters here
  double rho = params.get_double("rho");
  double gravity = params.get_double("gravity");

  if (armed_first_time_ && !baro_init_) {
    if (baro_count_ < 100) {
      init_static_ += msg->pressure;
      init_static_vector_.push_back(msg->pressure);
      input_.static_pres = 0;
      baro_count_ += 1;
    } else {
      init_static_ = std::accumulate(init_static_vector_.begin(), init_static_vector_.end(), 0.0)
        / init_static_vector_.size();
      baro_init_ = true;

      //Check that it got a good calibration.
      std::sort(init_static_vector_.begin(), init_static_vector_.end());
      float q1 = (init_static_vector_[24] + init_static_vector_[25]) / 2.0;
      float q3 = (init_static_vector_[74] + init_static_vector_[75]) / 2.0;
      float IQR = q3 - q1;
      float upper_bound = q3 + 2.0 * IQR;
      float lower_bound = q1 - 2.0 * IQR;
      for (int i = 0; i < 100; i++) {
        if (init_static_vector_[i] > upper_bound) {
          baro_init_ = false;
          baro_count_ = 0;
          init_static_vector_.clear();
          RCLCPP_WARN(this->get_logger(), "Bad baro calibration. Recalibrating");
          break;
        } else if (init_static_vector_[i] < lower_bound) {
          baro_init_ = false;
          baro_count_ = 0;
          init_static_vector_.clear();
          RCLCPP_WARN(this->get_logger(), "Bad baro calibration. Recalibrating");
          break;
        }
      }
    }
  } else {
    float static_pres_old = input_.static_pres;
    input_.static_pres = -msg->pressure + init_static_;

    float gate_gain = 1.35 * rho * gravity;
    if (input_.static_pres < static_pres_old - gate_gain) {
      input_.static_pres = static_pres_old - gate_gain;
    } else if (input_.static_pres > static_pres_old + gate_gain) {
      input_.static_pres = static_pres_old + gate_gain;
    }
  }
}

void estimator_base::airspeedCallback(const rosflight_msgs::msg::Airspeed::SharedPtr msg)
{
  // For readability, declare the parameters here
  double rho = params.get_double("rho");

  float diff_pres_old = input_.diff_pres;
  input_.diff_pres = msg->differential_pressure;

  float gate_gain = pow(5, 2) * rho / 2.0;
  if (input_.diff_pres < diff_pres_old - gate_gain) {
    input_.diff_pres = diff_pres_old - gate_gain;
  } else if (input_.diff_pres > diff_pres_old + gate_gain) {
    input_.diff_pres = diff_pres_old + gate_gain;
  }
}

void estimator_base::statusCallback(const rosflight_msgs::msg::Status::SharedPtr msg)
{
  if (!armed_first_time_ && msg->armed) armed_first_time_ = true;
}

} // namespace rosplane

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<rosplane::estimator_example>());

  return 0;
}
