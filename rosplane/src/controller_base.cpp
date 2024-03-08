#include "controller_base.hpp"
#include "controller_successive_loop.hpp"
#include "controller_total_energy.hpp"
#include <functional>
#include <rclcpp/logging.hpp>
#include <rosplane_msgs/msg/detail/controller_internals_debug__struct.hpp>

namespace rosplane
{

controller_base::controller_base()
    : Node("controller_base")
{

  // Advertise published topics.
  actuators_pub_ = this->create_publisher<rosflight_msgs::msg::Command>("command", 10);
  internals_pub_ = this->create_publisher<rosplane_msgs::msg::ControllerInternalsDebug>(
    "controller_inners_debug", 10);

  // Advertise subscribed topics and set bound callbacks.
  controller_commands_sub_ = this->create_subscription<rosplane_msgs::msg::ControllerCommands>(
    "controller_commands", 10, std::bind(&controller_base::controller_commands_callback, this, _1));
  vehicle_state_sub_ = this->create_subscription<rosplane_msgs::msg::State>(
    "estimated_state", 10, std::bind(&controller_base::vehicle_state_callback, this, _1));

  // This flag indicates whether the first set of commands have been received.
  command_recieved_ = false;

  // Declare the parameters for ROS2 param system.
  declare_parameters();

  // Set the values for the parameters, from the param file or use the deafault value.
  set_parameters();

  if (params_.roll_tuning_debug_override || params_.pitch_tuning_debug_override) {
    tuning_debug_sub_ = this->create_subscription<rosplane_msgs::msg::ControllerInternalsDebug>(
      "/tuning_debug", 10, std::bind(&controller_base::tuning_debug_callback, this, _1));
  }

  // Set the parameter callback, for when parameters are changed.
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&controller_base::parametersCallback, this, std::placeholders::_1));

  set_timer();
}

void controller_base::controller_commands_callback(
  const rosplane_msgs::msg::ControllerCommands::SharedPtr msg)
{

  // Set the flag that a command has been received.
  command_recieved_ = true;

  // Save the message to use in calculations.
  controller_commands_ = *msg;
}

void controller_base::vehicle_state_callback(const rosplane_msgs::msg::State::SharedPtr msg)
{

  // Save the message to use in calculations.
  vehicle_state_ = *msg;
}

void controller_base::tuning_debug_callback(
  const rosplane_msgs::msg::ControllerInternalsDebug::SharedPtr msg)
{
  // Save the message to use in calculations.
  tuning_debug_override_msg_ = *msg;
}

void controller_base::actuator_controls_publish()
{

  // Assemble inputs for the control algorithm.
  struct input_s input;
  input.h = -vehicle_state_.position[2];
  input.va = vehicle_state_.va;
  input.phi = vehicle_state_.phi;
  input.theta = vehicle_state_.theta;
  input.chi = vehicle_state_.chi;
  input.p = vehicle_state_.p;
  input.q = vehicle_state_.q;
  input.r = vehicle_state_.r;
  input.Va_c = controller_commands_.va_c;
  input.h_c = controller_commands_.h_c;
  input.chi_c = controller_commands_.chi_c;
  input.phi_ff = controller_commands_.phi_ff;

  struct output_s output;

  // If a command was received, begin control.
  if (command_recieved_ == true) {

    // Control based off of inputs and parameters.
    control(params_, input, output);

    // Convert control outputs to pwm.
    convert_to_pwm(output);

    rosflight_msgs::msg::Command actuators;

    // Find the current time, and save as a timestamp.
    rclcpp::Time now = this->get_clock()->now();

    // Attach the timestamp.
    actuators.header.stamp = now;

    // Do not ignore any of the actuators.
    actuators.ignore = 0;

    // Indicate that commands are for the actuators directly.
    actuators.mode = rosflight_msgs::msg::Command::MODE_PASS_THROUGH;

    // Package control efforts. If the output is infinite replace with 0.
    actuators.x = (std::isfinite(output.delta_a)) ? output.delta_a : 0.0f;
    actuators.y = (std::isfinite(output.delta_e)) ? output.delta_e : 0.0f;
    actuators.z = (std::isfinite(output.delta_r)) ? output.delta_r : 0.0f;
    actuators.f = (std::isfinite(output.delta_t)) ? output.delta_t : 0.0f;

    // Publish actuators.
    actuators_pub_->publish(actuators);

    // If there is a subscriber to the controller inners topic, publish the altitude zone and intermediate
    // control values, phi_c (commanded roll angle), and theta_c (commanded pitch angle).
    if (internals_pub_->get_subscription_count() > 0) {
      rosplane_msgs::msg::ControllerInternalsDebug inners;

      inners.header.stamp = now;

      inners.phi_c = output.phi_c;
      inners.theta_c = output.theta_c;
      switch (output.current_zone) {
        case alt_zones::TAKE_OFF:
          inners.alt_zone = inners.ZONE_TAKE_OFF;
          break;
        case alt_zones::CLIMB:
          inners.alt_zone = inners.ZONE_CLIMB;
          break;
        case alt_zones::ALTITUDE_HOLD:
          inners.alt_zone = inners.ZONE_ALTITUDE_HOLD;
          break;
        default:
          break;
      }
      inners.aux_valid = false;
      internals_pub_->publish(inners);
    }
  }
}

void controller_base::declare_parameters()
{

  // Declare each of the parameters, making it visible to the ROS2 param system.
  this->declare_parameter("alt_hz", params_.alt_hz);
  this->declare_parameter("alt_toz", params_.alt_toz);
  this->declare_parameter("tau", params_.tau);
  this->declare_parameter("c_kp", params_.c_kp);
  this->declare_parameter("c_kd", params_.c_kd);
  this->declare_parameter("c_ki", params_.c_ki);
  this->declare_parameter("r_kp", params_.r_kp);
  this->declare_parameter("r_kd", params_.r_kd);
  this->declare_parameter("r_ki", params_.r_ki);
  this->declare_parameter("p_kp", params_.p_kp);
  this->declare_parameter("p_kd", params_.p_kd);
  this->declare_parameter("p_ki", params_.p_ki);
  this->declare_parameter("p_ff", params_.p_ff);
  this->declare_parameter("a_t_kp", params_.a_t_kp);
  this->declare_parameter("a_t_kd", params_.a_t_kd);
  this->declare_parameter("a_t_ki", params_.a_t_ki);
  this->declare_parameter("a_kp", params_.a_kp);
  this->declare_parameter("a_kd", params_.a_kd);
  this->declare_parameter("a_ki", params_.a_ki);
  this->declare_parameter("l_kp", params_.l_kp);
  this->declare_parameter("l_kd", params_.l_kd);
  this->declare_parameter("l_ki", params_.l_ki);
  this->declare_parameter("e_kp", params_.e_kp);
  this->declare_parameter("e_kd", params_.e_kd);
  this->declare_parameter("e_ki", params_.e_ki);
  this->declare_parameter("trim_e", params_.trim_e);
  this->declare_parameter("trim_a", params_.trim_a);
  this->declare_parameter("trim_r", params_.trim_r);
  this->declare_parameter("trim_t", params_.trim_t);
  this->declare_parameter("max_e", params_.max_e);
  this->declare_parameter("max_a", params_.max_a);
  this->declare_parameter("max_r", params_.max_r);
  this->declare_parameter("max_t", params_.max_t);
  this->declare_parameter("pwm_rad_e", params_.pwm_rad_e);
  this->declare_parameter("pwm_rad_a", params_.pwm_rad_a);
  this->declare_parameter("pwm_rad_r", params_.pwm_rad_r);
  this->declare_parameter("max_takeoff_throttle", params_.max_takeoff_throttle);
  this->declare_parameter("mass", params_.mass);
  this->declare_parameter("gravity", params_.gravity);
  this->declare_parameter("pitch_tuning_debug_override", params_.pitch_tuning_debug_override);
  this->declare_parameter("roll_tuning_debug_override", params_.roll_tuning_debug_override);
  this->declare_parameter("max_roll", params_.max_roll);
  this->declare_parameter("frequency", params_.frequency);
}

void controller_base::set_parameters()
{

  // Get the parameters from the launch file, if given.
  // If not, use the default value defined in the header file.
  params_.alt_hz = this->get_parameter("alt_hz").as_double();
  params_.alt_toz = this->get_parameter("alt_toz").as_double();
  params_.tau = this->get_parameter("tau").as_double();
  params_.c_kp = this->get_parameter("c_kp").as_double();
  params_.c_kd = this->get_parameter("c_kd").as_double();
  params_.c_ki = this->get_parameter("c_ki").as_double();
  params_.r_kp = this->get_parameter("r_kp").as_double();
  params_.r_kd = this->get_parameter("r_kd").as_double();
  params_.r_ki = this->get_parameter("r_ki").as_double();
  params_.p_kp = this->get_parameter("p_kp").as_double();
  params_.p_kd = this->get_parameter("p_kd").as_double();
  params_.p_ki = this->get_parameter("p_ki").as_double();
  params_.p_ff = this->get_parameter("p_ff").as_double();
  params_.a_t_kp = this->get_parameter("a_t_kp").as_double();
  params_.a_t_kd = this->get_parameter("a_t_kd").as_double();
  params_.a_t_ki = this->get_parameter("a_t_ki").as_double();
  params_.a_kp = this->get_parameter("a_kp").as_double();
  params_.a_kd = this->get_parameter("a_kd").as_double();
  params_.a_ki = this->get_parameter("a_ki").as_double();
  params_.l_kp = this->get_parameter("l_kp").as_double();
  params_.l_kd = this->get_parameter("l_kd").as_double();
  params_.l_ki = this->get_parameter("l_ki").as_double();
  params_.e_kp = this->get_parameter("e_kp").as_double();
  params_.e_kd = this->get_parameter("e_kd").as_double();
  params_.e_ki = this->get_parameter("e_ki").as_double();
  params_.trim_e = this->get_parameter("trim_e").as_double();
  params_.trim_a = this->get_parameter("trim_a").as_double();
  params_.trim_r = this->get_parameter("trim_r").as_double();
  params_.trim_t = this->get_parameter("trim_t").as_double();
  params_.max_e = this->get_parameter("max_e").as_double();
  params_.max_a = this->get_parameter("max_a").as_double();
  params_.max_r = this->get_parameter("max_r").as_double();
  params_.max_t = this->get_parameter("max_t").as_double();
  params_.pwm_rad_e = this->get_parameter("pwm_rad_e").as_double();
  params_.pwm_rad_a = this->get_parameter("pwm_rad_a").as_double();
  params_.pwm_rad_r = this->get_parameter("pwm_rad_r").as_double();
  params_.max_takeoff_throttle = this->get_parameter("pwm_rad_r").as_double();
  params_.mass = this->get_parameter("mass").as_double();
  params_.gravity = this->get_parameter("gravity").as_double();
  params_.roll_tuning_debug_override = this->get_parameter("roll_tuning_debug_override").as_bool();
  params_.pitch_tuning_debug_override =
    this->get_parameter("pitch_tuning_debug_override").as_bool();
  params_.max_roll = this->get_parameter("max_roll").as_double();
  params_.frequency = this->get_parameter("frequency").as_int();
}

rcl_interfaces::msg::SetParametersResult
controller_base::parametersCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  // Check each parameter in the incoming vector of parameters to change, and change the appropriate parameter.
  for (const auto & param : parameters) {

    if (param.get_name() == "alt_hz") params_.alt_hz = param.as_double();
    else if (param.get_name() == "alt_toz")
      params_.alt_toz = param.as_double();
    else if (param.get_name() == "tau")
      params_.tau = param.as_double();
    else if (param.get_name() == "c_kp")
      params_.c_kp = param.as_double();
    else if (param.get_name() == "c_kd")
      params_.c_kd = param.as_double();
    else if (param.get_name() == "c_ki")
      params_.c_ki = param.as_double();
    else if (param.get_name() == "r_kp")
      params_.r_kp = param.as_double();
    else if (param.get_name() == "r_kd")
      params_.r_kd = param.as_double();
    else if (param.get_name() == "r_ki")
      params_.r_ki = param.as_double();
    else if (param.get_name() == "p_kp")
      params_.p_kp = param.as_double();
    else if (param.get_name() == "p_kd")
      params_.p_kd = param.as_double();
    else if (param.get_name() == "p_ki")
      params_.p_ki = param.as_double();
    else if (param.get_name() == "p_ff")
      params_.p_ff = param.as_double();
    else if (param.get_name() == "a_t_kp")
      params_.a_t_kp = param.as_double();
    else if (param.get_name() == "a_t_kd")
      params_.a_t_kd = param.as_double();
    else if (param.get_name() == "a_t_ki")
      params_.a_t_ki = param.as_double();
    else if (param.get_name() == "a_kp")
      params_.a_kp = param.as_double();
    else if (param.get_name() == "a_kd")
      params_.a_kd = param.as_double();
    else if (param.get_name() == "a_ki")
      params_.a_ki = param.as_double();
    else if (param.get_name() == "l_kp")
      params_.l_kp = param.as_double();
    else if (param.get_name() == "l_kd")
      params_.l_kd = param.as_double();
    else if (param.get_name() == "l_ki")
      params_.l_ki = param.as_double();
    else if (param.get_name() == "e_kp")
      params_.e_kp = param.as_double();
    else if (param.get_name() == "e_kd")
      params_.e_kd = param.as_double();
    else if (param.get_name() == "e_ki")
      params_.e_ki = param.as_double();
    else if (param.get_name() == "trim_e")
      params_.trim_e = param.as_double();
    else if (param.get_name() == "trim_a")
      params_.trim_a = param.as_double();
    else if (param.get_name() == "trim_r")
      params_.trim_r = param.as_double();
    else if (param.get_name() == "trim_t")
      params_.trim_t = param.as_double();
    else if (param.get_name() == "max_e")
      params_.max_e = param.as_double();
    else if (param.get_name() == "max_a")
      params_.max_a = param.as_double();
    else if (param.get_name() == "max_r")
      params_.max_r = param.as_double();
    else if (param.get_name() == "max_t")
      params_.max_t = param.as_double();
    else if (param.get_name() == "pwm_rad_e")
      params_.pwm_rad_e = param.as_double();
    else if (param.get_name() == "pwm_rad_a")
      params_.pwm_rad_a = param.as_double();
    else if (param.get_name() == "pwm_rad_r")
      params_.pwm_rad_r = param.as_double();
    else if (param.get_name() == "max_takeoff_throttle")
      params_.max_takeoff_throttle = param.as_double();
    else if (param.get_name() == "mass")
      params_.mass = param.as_double();
    else if (param.get_name() == "gravity")
      params_.gravity = param.as_double();
    else if (param.get_name() == "roll_tuning_debug_override")
      params_.roll_tuning_debug_override = param.as_bool();
    else if (param.get_name() == "pitch_tuning_debug_override")
      params_.pitch_tuning_debug_override = param.as_bool();
    else if (param.get_name() == "max_roll")
      params_.max_roll = param.as_double();
    else if (param.get_name() == "frequency") {
      params_.frequency = param.as_int();
      timer_->cancel();
      set_timer();
    } else {

      // If the parameter given doesn't match any of the parameters return false.
      result.successful = false;
      result.reason =
        "One of the parameters given does not is not a parameter of the controller node.";
      break;
    }
  }

  return result;
}

void controller_base::set_timer()
{
  auto timer_period =
    std::chrono::microseconds(static_cast<long long>(1.0 / params_.frequency * 1'000'000));

  // Set timer to trigger bound callback (actuator_controls_publish) at the given periodicity.
  timer_ = this->create_wall_timer(timer_period,
                                   std::bind(&controller_base::actuator_controls_publish,
                                             this)); // TODO add the period to the params.
}

void controller_base::convert_to_pwm(controller_base::output_s & output)
{

  // Multiply each control effort (in radians) by a scaling factor to a pwm.
  output.delta_e = output.delta_e
    * params_
        .pwm_rad_e; // TODO investigate why this is named "pwm". The actual scaling to pwm happens in rosflight_io.
  output.delta_a = output.delta_a * params_.pwm_rad_a;
  output.delta_r = output.delta_r * params_.pwm_rad_r;
}

} // namespace rosplane

int main(int argc, char * argv[])
{

  // Initialize ROS2 and then begin to spin control node.
  rclcpp::init(argc, argv);

  std::cout << argv[1] << std::endl;

  if (strcmp(argv[1], "total_energy") == 0) {
    auto node = std::make_shared<rosplane::controller_total_energy>();
    RCLCPP_INFO_STREAM(node->get_logger(), "Using total energy control.");
    rclcpp::spin(node);
  } else if (strcmp(argv[1], "default") == 0) {
    auto node = std::make_shared<rosplane::controller_successive_loop>();
    RCLCPP_INFO_STREAM(node->get_logger(), "Using default control.");
    rclcpp::spin(node);
  } else {
    auto node = std::make_shared<rosplane::controller_successive_loop>();
    RCLCPP_INFO_STREAM(node->get_logger(), "Invalid control type, using default control.");
    rclcpp::spin(node);
  }

  return 0;
}
