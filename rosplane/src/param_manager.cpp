#include "param_manager.hpp"

namespace rosplane
{

param_manager::param_manager(rclcpp::Node * node) : container_node_{node} {}

void param_manager::declare_param(std::string param_name, double value)
{
  // Insert the parameter into the parameter struct
  params_[param_name] = value;
  // Declare each of the parameters, making it visible to the ROS2 param system.
  container_node_->declare_parameter(param_name, value);
}

void param_manager::declare_param(std::string param_name, bool value)
{
  // Insert the parameter into the parameter struct
  params_[param_name] = value;
  // Declare each of the parameters, making it visible to the ROS2 param system.
  container_node_->declare_parameter(param_name, value);
}

void param_manager::declare_int(std::string param_name, int64_t value)
{
  // Insert the parameter into the parameter struct
  params_[param_name] = value;
  // Declare each of the parameters, making it visible to the ROS2 param system.
  container_node_->declare_parameter(param_name, value);
}

void param_manager::declare_param(std::string param_name, std::string value)
{
  // Insert the parameter into the parameter struct
  params_[param_name] = value;
  // Declare each of the parameters, making it visible to the ROS2 param system.
  container_node_->declare_parameter(param_name, value);
}

double param_manager::get_double(std::string param_name)
{
    return std::get<double>(params_[param_name]);
} 

bool param_manager::get_bool(std::string param_name)
{
  return std::get<bool>(params_[param_name]);
} 

int64_t param_manager::get_int(std::string param_name)
{
  return std::get<int64_t>(params_[param_name]);
} 

std::string param_manager::get_string(std::string param_name)
{
  return std::get<std::string>(params_[param_name]);
} 

void param_manager::set_parameters()
{

  // Get the parameters from the launch file, if given.
  // If not, use the default value defined in the header file.
  for (const auto& [key, value] : params_)
  {
    auto type = container_node_->get_parameter(key).get_type();
    if (type == rclcpp::ParameterType::PARAMETER_DOUBLE)
      params_[key] = container_node_->get_parameter(key).as_double();
    else if (type == rclcpp::ParameterType::PARAMETER_BOOL)
      params_[key] = container_node_->get_parameter(key).as_bool();
    else if (type == rclcpp::ParameterType::PARAMETER_INTEGER)
      params_[key] = container_node_->get_parameter(key).as_int();
    else if (type == rclcpp::ParameterType::PARAMETER_STRING)
      params_[key] = container_node_->get_parameter(key).as_string();
    else  
      RCLCPP_ERROR_STREAM(container_node_->get_logger(), "Unable to set parameter: " + key + ". Error casting parameter as double, int, string, or bool!");
  }
}

}   // namespace rosplane