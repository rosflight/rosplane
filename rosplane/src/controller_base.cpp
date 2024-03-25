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

  bool roll_tuning_debug_override = get_bool("roll_tuning_debug_override");
  bool pitch_tuning_debug_override = get_bool("pitch_tuning_debug_override");

  if (roll_tuning_debug_override || pitch_tuning_debug_override) {
    tuning_debug_sub_ = this->create_subscription<rosplane_msgs::msg::ControllerInternalsDebug>(
      "/tuning_debug", 10, std::bind(&controller_base::tuning_debug_callback, this, _1));
  }

  // Set the parameter callback, for when parameters are changed.
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&controller_base::parametersCallback, this, std::placeholders::_1));

  set_timer();
}

void controller_base::declare_parameters()
{
  // Declare default parameters associated with this controller, controller_base
  declare_param("roll_tuning_debug_override", false);
  declare_param("pitch_tuning_debug_override", false);
  declare_param("pwm_rad_e", 1.0);
  declare_param("pwm_rad_a", 1.0);
  declare_param("pwm_rad_r", 1.0);
  declare_int("frequency", 100);
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
    control(input, output);

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

void controller_base::declare_param(std::string param_name, double value)
{

  // Insert the parameter into the parameter struct
  params1_[param_name] = value;

  // Declare each of the parameters, making it visible to the ROS2 param system.
  this->declare_parameter(param_name, value);
}

void controller_base::declare_param(std::string param_name, bool value)
{

  // Insert the parameter into the parameter struct
  params1_[param_name] = value;

  // Declare each of the parameters, making it visible to the ROS2 param system.
  this->declare_parameter(param_name, value);
}

void controller_base::declare_int(std::string param_name, int64_t value)
{

  // Insert the parameter into the parameter struct
  params1_[param_name] = value;

  // Declare each of the parameters, making it visible to the ROS2 param system.
  this->declare_parameter(param_name, value);
}

void controller_base::declare_param(std::string param_name, std::string value)
{

  // Insert the parameter into the parameter struct
  params1_[param_name] = value;

  // Declare each of the parameters, making it visible to the ROS2 param system.
  this->declare_parameter(param_name, value);
}

void controller_base::set_parameters()
{

  // Get the parameters from the launch file, if given.
  // If not, use the default value defined in the header file.
  for (const auto& [key, value] : params1_)
  {
    auto type = this->get_parameter(key).get_type();
    if (type == rclcpp::ParameterType::PARAMETER_DOUBLE)
      params1_[key] = this->get_parameter(key).as_double();
    else if (type == rclcpp::ParameterType::PARAMETER_BOOL)
      params1_[key] = this->get_parameter(key).as_bool();
    else if (type == rclcpp::ParameterType::PARAMETER_INTEGER)
      params1_[key] = this->get_parameter(key).as_int();
    else if (type == rclcpp::ParameterType::PARAMETER_STRING)
      params1_[key] = this->get_parameter(key).as_string();
    else  
      RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to set parameter: " + key + ". Error casting parameter as double, int, string, or bool!");
  }

}

rcl_interfaces::msg::SetParametersResult 
controller_base::parametersCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  // Check each parameter in the incoming vector of parameters to change, and change the appropriate parameter.
  for (const auto & param : parameters) {
    
    // Check if the parameter is in the params object or return an error
    if (params1_.find(param.get_name()) == params1_.end()) {
      result.successful = false;
      result.reason =
        "One of the parameters given does not is not a parameter of the controller node. Parameter: " + param.get_name();
      break;
    }
    
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      params1_[param.get_name()] = param.as_double();
    else if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
      params1_[param.get_name()] = param.as_bool();
    else if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
      params1_[param.get_name()] = param.as_string();
    else
      RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to determine parameter type in controller. Type is " + std::to_string(param.get_type()));

  }

  return result;
}

double controller_base::get_double(std::string param_name)
{
  // try 
  // {
    return std::get<double>(params1_[param_name]);
  // }
  // catch (const std::bad_variant_access& e)
  // {
  //   RCLCPP_ERROR_STREAM(this->get_logger(), "Error when accessing " + param_name + " parameter: " + e.what());
  // }
} 

bool controller_base::get_bool(std::string param_name)
{
  return std::get<bool>(params1_[param_name]);
} 

int64_t controller_base::get_int(std::string param_name)
{
  return std::get<int64_t>(params1_[param_name]);
} 

std::string controller_base::get_string(std::string param_name)
{
  return std::get<std::string>(params1_[param_name]);
} 

void controller_base::set_timer()
{

  int64_t frequency = get_int("frequency");
  auto timer_period =
    std::chrono::microseconds(static_cast<long long>(1.0 / frequency * 1'000'000));

  // Set timer to trigger bound callback (actuator_controls_publish) at the given periodicity.
  timer_ = this->create_wall_timer(timer_period,
                                   std::bind(&controller_base::actuator_controls_publish,
                                             this)); // TODO add the period to the params.
}

void controller_base::convert_to_pwm(controller_base::output_s & output)
{

  // Assign parameters from parameters object
  double pwm_rad_e = get_double("pwm_rad_e");
  double pwm_rad_a = get_double("pwm_rad_a");
  double pwm_rad_r = get_double("pwm_rad_r");
  // double test = get_double("frequency");

  // Multiply each control effort (in radians) by a scaling factor to a pwm.
  // TODO investigate why this is named "pwm". The actual scaling to pwm happens in rosflight_io.
  output.delta_e = output.delta_e * pwm_rad_e; 
  output.delta_a = output.delta_a * pwm_rad_a;
  output.delta_r = output.delta_r * pwm_rad_r;
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
