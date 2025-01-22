#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>

#include "estimator_continuous_discrete.hpp"
#include "estimator_ros.hpp"

namespace rosplane
{

EstimatorROS::EstimatorROS()
    : Node("estimator_ros")
    , params_(this)
    , params_initialized_(false)
{
  vehicle_state_pub_ = this->create_publisher<rosplane_msgs::msg::State>("estimated_state", 10);

  gnss_fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    gnss_fix_topic_, 10, std::bind(&EstimatorROS::gnssFixCallback, this, std::placeholders::_1));
  gnss_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    gnss_vel_topic_, 10, std::bind(&EstimatorROS::gnssVelCallback, this, std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, 10, std::bind(&EstimatorROS::imuCallback, this, std::placeholders::_1));
  baro_sub_ = this->create_subscription<rosflight_msgs::msg::Barometer>(
    baro_topic_, 10, std::bind(&EstimatorROS::baroAltCallback, this, std::placeholders::_1));
  airspeed_sub_ = this->create_subscription<rosflight_msgs::msg::Airspeed>(
    airspeed_topic_, 10, std::bind(&EstimatorROS::airspeedCallback, this, std::placeholders::_1));
  status_sub_ = this->create_subscription<rosflight_msgs::msg::Status>(
    status_topic_, 10, std::bind(&EstimatorROS::statusCallback, this, std::placeholders::_1));

  init_static_ = 0;
  baro_count_ = 0;
  armed_first_time_ = false;
  baro_init_ = false;
  gps_init_ = false;

  // Set the parameter callback, for when parameters are changed.
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&EstimatorROS::parametersCallback, this, std::placeholders::_1));

  // Declare and set parameters with the ROS2 system
  declare_parameters();
  params_.set_parameters();

  params_initialized_ = true;

  std::filesystem::path rosplane_dir = ament_index_cpp::get_package_share_directory("rosplane");

  std::filesystem::path params_dir = "params";
  std::filesystem::path params_file = "anaconda_autopilot_params.yaml";
  std::filesystem::path full_path = rosplane_dir / params_dir / params_file;

  param_filepath_ = full_path.string();

  input_.diff_pres = 0.0; // Initalize the differential_pressure measurement to zero.
  input_.static_pres = 0.0; // Initalize the differential_pressure measurement to zero.

  set_timer();
}

void EstimatorROS::declare_parameters()
{
  params_.declare_double("estimator_update_frequency", 100.0);
  params_.declare_double("rho", 1.225);
  params_.declare_double("gravity", 9.8);
  params_.declare_double("gps_ground_speed_threshold",
                         0.3); // TODO: this is a magic number. What is it determined from?
  params_.declare_double("baro_measurement_gate",
                         1.35); // TODO: this is a magic number. What is it determined from?
  params_.declare_double("airspeed_measurement_gate",
                         5.0); // TODO: this is a magic number. What is it determined from?
  params_.declare_int("baro_calibration_count",
                      100); // TODO: this is a magic number. What is it determined from?
  params_.declare_double("baro_calibration_val", 0.0);
  params_.declare_double("init_lat", 0.0);
  params_.declare_double("init_lon", 0.0);
  params_.declare_double("init_alt", 0.0);
}

void EstimatorROS::set_timer()
{
  double frequency = params_.get_double("estimator_update_frequency");

  update_period_ = std::chrono::microseconds(static_cast<long long>(1.0 / frequency * 1'000'000));
  update_timer_ = this->create_wall_timer(update_period_, std::bind(&EstimatorROS::update, this));
}

rcl_interfaces::msg::SetParametersResult
EstimatorROS::parametersCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  bool success = params_.set_parameters_callback(parameters);
  if (!success) {
    result.successful = false;
    result.reason = "One of the parameters given is not a parameter of the estimator node.";
  }

  // Check to see if the timer period was changed. If it was, recreate the timer with the new period
  if (params_initialized_ && success) {
    std::chrono::microseconds curr_period = std::chrono::microseconds(
      static_cast<long long>(1.0 / params_.get_double("estimator_update_frequency") * 1'000'000));
    if (update_period_ != curr_period) {
      update_timer_->cancel();
      set_timer();
    }
  }

  return result;
}

void EstimatorROS::update()
{
  Output output;

  if (armed_first_time_) {
    estimate(input_, output);
  } else {
    output.pn = output.pe = output.h = 0;
    output.phi = output.theta = output.psi = 0;
    output.alpha = output.beta = output.chi = 0;
    output.p = output.q = output.r = 0;
    output.va = 0;
  }

  input_.gps_new = false;

  rosplane_msgs::msg::State msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = 1; // Denotes global frame

  msg.position[0] = output.pn;
  msg.position[1] = output.pe;
  msg.position[2] = -output.h; // Nominal output is alt. For NED the alt must be inverted.
  if (gps_init_) {
    msg.initial_lat = init_lat_;
    msg.initial_lon = init_lon_;
    msg.initial_alt = init_alt_;
  }
  msg.va = output.va;
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

  msg.u = output.va * cos(output.theta);
  msg.v = 0;
  msg.w = output.va * sin(output.theta);

  msg.psi_deg = fmod(output.psi, 2.0 * M_PI) * 180 / M_PI; //-360 to 360
  msg.psi_deg += (msg.psi_deg < -180 ? 360 : 0);
  msg.psi_deg -= (msg.psi_deg > 180 ? 360 : 0);
  msg.chi_deg = fmod(output.chi, 2.0 * M_PI) * 180 / M_PI; //-360 to 360
  msg.chi_deg += (msg.chi_deg < -180 ? 360 : 0);
  msg.chi_deg -= (msg.chi_deg > 180 ? 360 : 0);

  vehicle_state_pub_->publish(msg);
}

void EstimatorROS::gnssFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  bool has_fix = msg->status.status
    >= sensor_msgs::msg::NavSatStatus::STATUS_FIX; // Higher values refer to augmented fixes
  if (!has_fix || !std::isfinite(msg->latitude)) {
    input_.gps_new = false;
    return;
  }
  if (!gps_init_ && has_fix) {
    gps_init_ = true;
    init_alt_ = msg->altitude;
    init_lat_ = msg->latitude;
    init_lon_ = msg->longitude;
    saveParameter("init_lat", init_lat_);
    saveParameter("init_lon", init_lon_);
    saveParameter("init_alt", init_alt_);
  } else {
    input_.gps_n = EARTH_RADIUS * (msg->latitude - init_lat_) * M_PI / 180.0;
    input_.gps_e =
      EARTH_RADIUS * cos(init_lat_ * M_PI / 180.0) * (msg->longitude - init_lon_) * M_PI / 180.0;
    input_.gps_h = msg->altitude - init_alt_;
    input_.gps_new = true;
  }
}

void EstimatorROS::gnssVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  // Rename parameter here for clarity
  double ground_speed_threshold = params_.get_double("gps_ground_speed_threshold");

  double v_n = msg->twist.linear.x;
  double v_e = msg->twist.linear.y;
  //  double v_d = msg->twist.linear.z; // This variable was unused.
  double ground_speed = sqrt(v_n * v_n + v_e * v_e);
  double course =
    atan2(v_e, v_n); //Does this need to be in a specific range? All uses seem to accept anything.
  input_.gps_Vg = ground_speed;
  if (ground_speed > ground_speed_threshold)
    input_.gps_course = course;
}

void EstimatorROS::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  input_.accel_x = msg->linear_acceleration.x;
  input_.accel_y = msg->linear_acceleration.y;
  input_.accel_z = msg->linear_acceleration.z;

  input_.gyro_x = msg->angular_velocity.x;
  input_.gyro_y = msg->angular_velocity.y;
  input_.gyro_z = msg->angular_velocity.z;
}

void EstimatorROS::baroAltCallback(const rosflight_msgs::msg::Barometer::SharedPtr msg)
{
  // For readability, declare the parameters here
  double rho = params_.get_double("rho");
  double gravity = params_.get_double("gravity");
  double gate_gain_constant = params_.get_double("baro_measurement_gate");
  double baro_calib_count = params_.get_int("baro_calibration_count");

  if (armed_first_time_ && !baro_init_) {
    if (baro_count_ < baro_calib_count) {
      init_static_ += msg->pressure;
      init_static_vector_.push_back(msg->pressure);
      input_.static_pres = 0;
      baro_count_ += 1;
    } else {
      init_static_ = std::accumulate(init_static_vector_.begin(), init_static_vector_.end(), 0.0)
        / init_static_vector_.size();
      baro_init_ = true;
      saveParameter("baro_calibration_val", init_static_);

      //Check that it got a good calibration.
      std::sort(init_static_vector_.begin(), init_static_vector_.end());
      float q1 = (init_static_vector_[24] + init_static_vector_[25]) / 2.0;
      float q3 = (init_static_vector_[74] + init_static_vector_[75]) / 2.0;
      float IQR = q3 - q1;
      float upper_bound = q3 + 2.0 * IQR;
      float lower_bound = q1 - 2.0 * IQR;
      for (int i = 0; i < baro_calib_count; i++) {
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

    float gate_gain = gate_gain_constant * rho * gravity;
    if (input_.static_pres < static_pres_old - gate_gain) {
      input_.static_pres = static_pres_old - gate_gain;
    } else if (input_.static_pres > static_pres_old + gate_gain) {
      input_.static_pres = static_pres_old + gate_gain;
    }
  }
}

void EstimatorROS::airspeedCallback(const rosflight_msgs::msg::Airspeed::SharedPtr msg)
{
  // For readability, declare the parameters here
  double rho = params_.get_double("rho");
  double gate_gain_constant = params_.get_double("airspeed_measurement_gate");

  float diff_pres_old = input_.diff_pres;
  input_.diff_pres = msg->differential_pressure;

  float gate_gain = pow(gate_gain_constant, 2) * rho / 2.0;
  if (input_.diff_pres < diff_pres_old - gate_gain) {
    input_.diff_pres = diff_pres_old - gate_gain;
  } else if (input_.diff_pres > diff_pres_old + gate_gain) {
    input_.diff_pres = diff_pres_old + gate_gain;
  }
}

void EstimatorROS::statusCallback(const rosflight_msgs::msg::Status::SharedPtr msg)
{
  if (!armed_first_time_ && msg->armed)
    armed_first_time_ = true;
}

void EstimatorROS::saveParameter(std::string param_name, double param_val)
{

  YAML::Node param_yaml_file = YAML::LoadFile(param_filepath_);

  if (param_yaml_file["estimator"]["ros__parameters"][param_name]) {
    param_yaml_file["estimator"]["ros__parameters"][param_name] = param_val;
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Parameter [" << param_name << "] is not in parameter file.");
  }

  std::ofstream fout(param_filepath_);
  fout << param_yaml_file;
}

} // namespace rosplane

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);

  char* use_params;
  if (argc >= 2) {
    use_params = argv[1];
  }

  if (!strcmp(use_params, "true")) {
    rclcpp::spin(std::make_shared<rosplane::EstimatorContinuousDiscrete>(use_params));
  } else if (strcmp(use_params, "false")) // If the string is not true or false print error.
  {
    auto estimator_node = std::make_shared<rosplane::EstimatorContinuousDiscrete>();
    RCLCPP_WARN(estimator_node->get_logger(),
                "Invalid option for seeding estimator, defaulting to unseeded.");
    rclcpp::spin(estimator_node);
  } else {
    rclcpp::spin(std::make_shared<rosplane::EstimatorContinuousDiscrete>());
  }

  return 0;
}
