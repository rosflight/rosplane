//
// Created by brandon on 7/18/24.
//

#ifndef LOGGER_INTERFACE_HPP
#define LOGGER_INTERFACE_HPP

#include <sstream>

namespace rosplane
{
class LoggerInterface
{
public:
  virtual ~LoggerInterface() = default;

  virtual void debug(const std::stringstream& message) = 0;
  virtual void info(const std::stringstream& message) = 0;
  virtual void warn(const std::stringstream& message) = 0;
  virtual void error(const std::stringstream& message) = 0;
  virtual void fatal(const std::stringstream& message) = 0;
};
}

#endif //LOGGER_INTERFACE_HPP
