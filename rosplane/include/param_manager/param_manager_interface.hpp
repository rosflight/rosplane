#ifndef PARAM_MANAGER_INTERFACE_HPP
#define PARAM_MANAGER_INTERFACE_HPP

#include <string>

namespace rosplane
{
class ParamMangerInterface
{
public:
  virtual ~ParamMangerInterface() = default;

  /**
   * Helper function to access parameter values of type double stored in param_manager object
   * @return Double value of the parameter
   */
  virtual double get_double(std::string param_name) = 0;

  /**
   * Helper function to access parameter values of type bool stored in param_manager object
   * @return Bool value of the parameter
   */
  virtual bool get_bool(std::string param_name) = 0;

  /**
   * Helper function to access parameter values of type integer stored in param_manager object
   * @return Integer value of the parameter
   */
  virtual int64_t get_int(std::string param_name) = 0;

  /**
   * Helper function to access parameter values of type string stored in param_manager object
   * @return String value of the parameter
   */
  virtual std::string get_string(std::string param_name) = 0;

  /**
   * Helper function to declare parameters in the param_manager object
   * Inserts a parameter into the parameter object and declares it with the ROS system
   */
  virtual void declare_double(std::string param_name, double value) = 0;

  /**
   * Helper function to declare parameters in the param_manager object
   * Inserts a parameter into the parameter object and declares it with the ROS system
   */
  virtual void declare_bool(std::string param_name, bool value) = 0;

  /**
   * Helper function to declare parameters in the param_manager object
   * Inserts a parameter into the parameter object and declares it with the ROS system
   */
  virtual void declare_int(std::string param_name, int64_t value) = 0;

  /**
   * Helper function to declare parameters in the param_manager object
   * Inserts a parameter into the parameter object and declares it with the ROS system
   */
  virtual void declare_string(std::string param_name, std::string value) = 0;

  /**
   * This sets the parameters with the values in the params_ object from the supplied parameter file, or sets them to
   * the default if no value is given for a parameter.
   */
  virtual void set_parameters() = 0;

  /**
   * This function sets a previously declared parameter to a new value in both the parameter object
   * and the ROS system.
   */
  virtual void set_double(std::string param_name, double value) = 0;

  /**
   * This function sets a previously declared parameter to a new value in both the parameter object
   * and the ROS system.
   */
  virtual void set_bool(std::string param_name, bool value) = 0;

  /**
   * This function sets a previously declared parameter to a new value in both the parameter object
   * and the ROS system.
   */
  virtual void set_int(std::string param_name, int64_t value) = 0;

  /**
   * This function sets a previously declared parameter to a new value in both the parameter object
   * and the ROS system.
   */
  virtual void set_string(std::string param_name, std::string value) = 0;
};
} // namespace rosplane

#endif //PARAM_MANAGER_INTERFACE_HPP
