//
// Created by brandon on 7/18/24.
//

#ifndef LOGGER_ROS_HPP
#define LOGGER_ROS_HPP

#include <rclcpp/rclcpp.hpp>

#include "logger_interface.hpp"

namespace rosplane
{
class LoggerROS : public LoggerInterface
{
public:
  LoggerROS(rclcpp::Node & node);

  void debug(const std::stringstream& message) override;
  void info(const std::stringstream& message) override;
  void warn(const std::stringstream& message) override;
  void error(const std::stringstream& message) override;
  void fatal(const std::stringstream& message) override;

private:
  rclcpp::Node & node_;
};
} // namespace rosplane

#endif //LOGGER_ROS_HPP
