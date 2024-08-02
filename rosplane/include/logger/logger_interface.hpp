#ifndef LOGGER_INTERFACE_HPP
#define LOGGER_INTERFACE_HPP

#include <string>

namespace rosplane
{
class LoggerInterface
{
public:
  virtual ~LoggerInterface() = default;

  virtual void debug(const std::string & message) = 0;
  virtual void info(const std::string & message) = 0;
  virtual void warn(const std::string & message) = 0;
  virtual void error(const std::string & message) = 0;
  virtual void fatal(const std::string & message) = 0;
};
}

#endif //LOGGER_INTERFACE_HPP
