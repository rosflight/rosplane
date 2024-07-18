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

  void debug(const std::string& message) override;
  void info(const std::string& message) override;
  void warn(const std::string& message) override;
  void error(const std::string& message) override;
  void fatal(const std::string& message) override;

private:
  rclcpp::Node & node_;
};
} // namespace rosplane

#endif //LOGGER_ROS_HPP
