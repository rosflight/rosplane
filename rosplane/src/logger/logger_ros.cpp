#include "logger/logger_ros.hpp"

namespace rosplane
{
 LoggerROS::LoggerROS(rclcpp::Node & node) : node_(node) {}

void LoggerROS::debug(const std::string& message)
{
  RCLCPP_DEBUG_STREAM(node_.get_logger(), message);
}

void LoggerROS::info(const std::string& message)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), message);
}

void LoggerROS::warn(const std::string& message)
{
  RCLCPP_WARN_STREAM(node_.get_logger(), message);
}

void LoggerROS::error(const std::string & message)
{
  RCLCPP_ERROR_STREAM(node_.get_logger(), message);
}

void LoggerROS::fatal(const std::string & message)
{
  RCLCPP_FATAL_STREAM(node_.get_logger(), message);
}

} // namespace rosplane
