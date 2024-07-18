//
// Created by brandon on 7/18/24.
//

#include "logger/logger_ros.hpp"

namespace rosplane
{
 LoggerROS::LoggerROS(rclcpp::Node & node) : node_(node) {}

void LoggerROS::debug(const std::stringstream& message)
{
  RCLCPP_DEBUG_STREAM(node_.get_logger(), message.str());
}

void LoggerROS::info(const std::stringstream & message)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), message.str());
}

void LoggerROS::warn(const std::stringstream & message)
{
  RCLCPP_WARN_STREAM(node_.get_logger(), message.str());
}

void LoggerROS::error(const std::stringstream & message)
{
  RCLCPP_ERROR_STREAM(node_.get_logger(), message.str());
}

void LoggerROS::fatal(const std::stringstream & message)
{
  RCLCPP_FATAL_STREAM(node_.get_logger(), message.str());
}

} // namespace rosplane
