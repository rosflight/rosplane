#include "ekf/estimator_ros.hpp"
#include <fstream>
#include <rclcpp/logging.hpp>

namespace rosplane
{

EstimatorROS::EstimatorROS()
    : Node("estimator"), params_(this), params_initialized_(false)
{
  vehicle_state_pub_ = this->create_publisher<rosplane_msgs::msg::State>("estimated_state", 10);

  gnss_sub_ = this->create_subscription<rosflight_msgs::msg::GNSS>(
    gnss_fix_topic_, 10, std::bind(&EstimatorROS::gnssCallback, this, std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, 10, std::bind(&EstimatorROS::imuCallback, this, std::placeholders::_1));
  baro_sub_ = this->create_subscription<rosflight_msgs::msg::Barometer>(
    baro_topic_, 10, std::bind(&EstimatorROS::baroAltCallback, this, std::placeholders::_1));
  airspeed_sub_ = this->create_subscription<rosflight_msgs::msg::Airspeed>(
    airspeed_topic_, 10, std::bind(&EstimatorROS::airspeedCallback, this, std::placeholders::_1));
  status_sub_ = this->create_subscription<rosflight_msgs::msg::Status>(
    status_topic_, 10, std::bind(&EstimatorROS::statusCallback, this, std::placeholders::_1));
  magnetometer_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
    magnetometer_topic_, 10, std::bind(&EstimatorROS::magnetometerCallback, this, std::placeholders::_1));

  init_static_ = 0;
  baro_count_ = 0;
  armed_first_time_ = false;
  baro_init_ = false;
  gps_init_ = false;
  has_fix_ = false;

  // Set the parameter callback, for when parameters are changed.
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&EstimatorROS::parametersCallback, this, std::placeholders::_1));

  // Declare and set parameters with the ROS2 system
  declare_parameters();
  params_.set_parameters();

  params_initialized_ = true;
  
  std::filesystem::path workspace_dir = ament_index_cpp::get_package_share_directory("rosplane");
  std::filesystem::path params_dir = "params";
  std::filesystem::path hotstart_file = "hotstart";

  std::filesystem::path install_hotstart_dir = workspace_dir / params_dir / hotstart_file;
 
  // Remove the install and share directories from the path so the hotstart file goes into
  // the main directory so it is easily found.
  for (auto dir : install_hotstart_dir) {
    if (dir.string() == "install") continue;
    if (dir.string() == "share") continue;
    hotstart_path_ /= dir;
  }

  if (params_.get_bool("hotstart_estimator")) {
    hotstart();
  }

  set_timer();
}

void EstimatorROS::declare_parameters()
{
  params_.declare_double("estimator_update_frequency", 390.0);
  params_.declare_double("rho", NOT_IN_USE); // TODO: UPDATE THE PARAMS FILE TO NOT_IN_USE
  params_.declare_double("gravity", 9.81);
  params_.declare_double("gps_ground_speed_threshold", 0.3);  // TODO: this is a magic number. What is it determined from?
  params_.declare_double("baro_measurement_gate", 1.35);  // TODO: this is a magic number. What is it determined from?
  params_.declare_double("airspeed_measurement_gate", 5.0);  // TODO: this is a magic number. What is it determined from?
  params_.declare_int("baro_calibration_count", 100);  // TODO: this is a magic number. What is it determined from?
  params_.declare_int("min_gnss_fix_type", 3);
  params_.declare_double("baro_calibration_val", 0.0);
  params_.declare_bool("hotstart_estimator", false);
}

void EstimatorROS::hotstart()
{
  std::ifstream in(hotstart_path_.string());

  in >> init_lat_;
  in >> init_lon_;
  in >> init_alt_;
  in >> init_static_;
  in >> rho_;
}

void EstimatorROS::saveInitConditions()
{
  std::ofstream out(hotstart_path_.string());

  out << init_lat_ << " ";
  out << init_lon_ << " ";
  out << init_alt_ << " ";
  out << init_static_ << " ";
  out << rho_;
}

void EstimatorROS::set_timer() {
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
    result.reason =
      "One of the parameters given is not a parameter of the estimator node.";
  }
  else {
    parameter_changed = true;
  }

  // Check to see if the timer period was changed. If it was, recreate the timer with the new period
  if (params_initialized_ && success) {
    std::chrono::microseconds curr_period = std::chrono::microseconds(static_cast<long long>(1.0 / params_.get_double("estimator_update_frequency") * 1'000'000));
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
  /*output.pn = output.pe = output.pd = 0;*/
  /*output.phi = output.theta = output.psi = 0;*/
  /*output.vn = output.ve = output.vd = 0;*/
  /*output.p = output.q = output.r = 0;*/
  /*output.vg = 0;*/
  /*output.bx = output.by = output.bz = 0;*/
  /*output.wn = output.we = 0;*/
  /*output.alpha = output.beta = 0;*/
  /*output.va = output.vg;*/
  /*output.chi = output.psi;*/
  /*Eigen::Quaternionf init(1.0, 0.0, 0.0, 0.0);*/
  /*output.quat = init;*/

  if (armed_first_time_) {
    estimate(input_, output);
  }


  if (!init_conds_saved_ && baro_init_ && gps_init_ && !params_.get_bool("hotstart_estimator")) {
    saveInitConditions();
    init_conds_saved_ = true;
  }

  input_.gps_new = false;
  
  // Create estimated state message and publish it.
  rosplane_msgs::msg::State msg = rosplane_msgs::msg::State();
  msg.header.stamp = this->get_clock()->now();

  msg.position[0] = output.pn;
  msg.position[1] = output.pe;
  msg.position[2] = output.pd;
  msg.u = output.vx;
  msg.v = output.vy;
  msg.w = output.vz;
  msg.phi = output.phi;
  msg.theta = output.theta;
  msg.psi = output.psi;
  msg.p = output.p;
  msg.q = output.q;
  msg.r = output.r;
  msg.bx = output.bx;
  msg.by = output.by;
  msg.bz = output.bz;
  msg.initial_alt = init_alt_;
  msg.initial_lat = init_lat_;
  msg.initial_lon = init_lon_;
  msg.va = output.va;
  msg.beta = output.vx / output.va; // TODO: the ve should be compensated for wind.
  msg.alpha = 0.0; // TODO: Dr. McLain sent me a better way to calculate this.
  msg.chi = output.chi;
  msg.wn = output.wn;
  msg.we = output.we;

  // Fill in the quaternion
  msg.quat[0] = output.quat.w();
  msg.quat[1] = output.quat.x();
  msg.quat[2] = output.quat.y();
  msg.quat[3] = output.quat.z();
  msg.quat_valid = true;

  vehicle_state_pub_->publish(msg);
}

void EstimatorROS::gnssCallback(const rosflight_msgs::msg::GNSS::SharedPtr msg)
{
  int min_fix_type = params_.get_int("min_gnss_fix_type");

  // Convert msg to standard DDS and m/s.
  float msg_lat = msg->lat;
  float msg_lon = msg->lon;
  float msg_height = msg->alt;
  
  float msg_vel_n = msg->vel_n;
  float msg_vel_e = msg->vel_e;
  float msg_vel_d = msg->vel_d;

  has_fix_ = msg->fix_type >= min_fix_type; 
  
  if (!has_fix_ || !std::isfinite(msg->lat)) {
    input_.gps_new = false;
    return;
  }
  if (!gps_init_ && has_fix_) {
    gps_init_ = true;

    if (!params_.get_bool("hotstart_estimator")) {
      init_alt_ = msg_height;
      init_lat_ = msg_lat;
      init_lon_ = msg_lon;

      // Calculate the air density using the standard atmospheric model.
      double pressure_at_alt = 101325.0f * (float) pow((1 - 2.25694e-5 * init_alt_), 5.2553);
      rho_ = 1.225 * pow(pressure_at_alt / 101325.0, 0.809736894596450);
      
      // If the parameter is in use override the pressure at altitude calculation.
      float rho = params_.get_double("rho");
      if (rho > 0) {
        rho_ = rho;
      }
    }
  } else {
    input_.gps_lat = msg_lat;
    input_.gps_lon = msg_lon;
    input_.gps_alt = msg_height;

    // Convert UTC time into broken down format
    std::time_t time = msg->header.stamp.sec;
    const std::tm *broken_down_time = std::localtime(&time);
    input_.gps_year = broken_down_time->tm_year + 1900;
    input_.gps_month = broken_down_time->tm_mon;
    input_.gps_day = broken_down_time->tm_mday;

    input_.gps_n = EARTH_RADIUS * (msg_lat - init_lat_) * M_PI / 180.0;
    input_.gps_e =
      EARTH_RADIUS * cos(init_lat_ * M_PI / 180.0) * (msg_lon - init_lon_) * M_PI / 180.0;
    input_.gps_h = msg_height - init_alt_;
    input_.gps_new = true;

    double vg_when_course_valid = params_.get_double("gps_ground_speed_threshold");

    double ground_speed = sqrt(msg_vel_n * msg_vel_n + msg_vel_e * msg_vel_e);
    double course = atan2(msg_vel_e, msg_vel_n); 

    input_.gps_vg = ground_speed;
    input_.gps_vn = msg_vel_n;
    input_.gps_ve = msg_vel_e;
    input_.gps_vd = msg_vel_d;

    if (ground_speed > vg_when_course_valid)
      input_.gps_course = course;
  }
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
  double gravity = params_.get_double("gravity");
  double gate_gain_constant = params_.get_double("baro_measurement_gate");

  new_baro_ = true;

  if (armed_first_time_ && !baro_init_) {
    update_barometer_calibration(msg);
  } else {
    // Save the barometer pressure, cap the maximum change registered.
    float static_pres_old = input_.static_pres;
    input_.static_pres = -msg->pressure + init_static_;

    float gate_gain = gate_gain_constant * rho_ * gravity;
    if (input_.static_pres < static_pres_old - gate_gain) {
      input_.static_pres = static_pres_old - gate_gain;
    } else if (input_.static_pres > static_pres_old + gate_gain) {
      input_.static_pres = static_pres_old + gate_gain;
    }
  }
}

void EstimatorROS::update_barometer_calibration(const rosflight_msgs::msg::Barometer::SharedPtr msg)
{
  double baro_calib_count = params_.get_int("baro_calibration_count");

  if (baro_count_ < baro_calib_count) {
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
}

void EstimatorROS::airspeedCallback(const rosflight_msgs::msg::Airspeed::SharedPtr msg)
{

  if (msg->differential_pressure < 0.f) {
    return;
  }
  double gate_gain_constant = params_.get_double("airspeed_measurement_gate");

  float diff_pres_old = input_.diff_pres;
  input_.diff_pres = msg->differential_pressure;

  float gate_gain = pow(gate_gain_constant, 2) * rho_ / 2.0;
  if (input_.diff_pres < diff_pres_old - gate_gain) {
    input_.diff_pres = diff_pres_old - gate_gain;
  } else if (input_.diff_pres > diff_pres_old + gate_gain) {
    input_.diff_pres = diff_pres_old + gate_gain;
  }

  new_diff_ = true;
}

void EstimatorROS::magnetometerCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
{
  new_mag_ = true;

  input_.mag_x = msg->magnetic_field.x;
  input_.mag_y = msg->magnetic_field.y;
  input_.mag_z = msg->magnetic_field.z;
}

void EstimatorROS::statusCallback(const rosflight_msgs::msg::Status::SharedPtr msg)
{
  if (!armed_first_time_ && msg->armed) armed_first_time_ = true;
}

} // namespace rosplane
