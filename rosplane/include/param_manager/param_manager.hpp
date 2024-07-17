/**
 * @file param_manager.hpp
 * 
 * Manager for the parameters in ROSplane. Includes helper functions to set and get parameters
 * 
 * @author Jacob Moore <jacobmoor2@gmail.com>
*/

#ifndef PARAM_MANAGER_H
#define PARAM_MANAGER_H

#include <variant>

#include <rclcpp/rclcpp.hpp>

namespace rosplane
{

class ParamManager
{
public:
  /**
   * Public constructor
   * 
   * @param node: the ROS2 node that has this parameter object. Used to poll the ROS2 parameters
  */
  ParamManager(rclcpp::Node * node);

  /**
   * Helper function to access parameter values of type double stored in param_manager object
   * @return Double value of the parameter
  */
  double get_double(std::string param_name);

  /**
   * Helper function to access parameter values of type bool stored in param_manager object
   * @return Bool value of the parameter
  */
  bool get_bool(std::string param_name);

  /**
   * Helper function to access parameter values of type integer stored in param_manager object
   * @return Integer value of the parameter
  */
  int64_t get_int(std::string param_name);

  /**
   * Helper function to access parameter values of type string stored in param_manager object
   * @return String value of the parameter
  */
  std::string get_string(std::string param_name);

  /**
   * Helper function to declare parameters in the param_manager object
   * Inserts a parameter into the parameter object and declares it with the ROS system
  */
  void declare_double(std::string param_name, double value);

  /**
   * Helper function to declare parameters in the param_manager object
   * Inserts a parameter into the parameter object and declares it with the ROS system
  */
  void declare_bool(std::string param_name, bool value);

  /**
   * Helper function to declare parameters in the param_manager object
   * Inserts a parameter into the parameter object and declares it with the ROS system
  */
  void declare_int(std::string param_name, int64_t value);

  /**
   * Helper function to declare parameters in the param_manager object
   * Inserts a parameter into the parameter object and declares it with the ROS system
  */
  void declare_string(std::string param_name, std::string value);

  /**
   * This sets the parameters with the values in the params_ object from the supplied parameter file, or sets them to
   * the default if no value is given for a parameter.
   */
  void set_parameters();

  /**
   * This function sets a previously declared parameter to a new value in both the parameter object
   * and the ROS system.
   */
  void set_double(std::string param_name, double value);

  /**
   * This function sets a previously declared parameter to a new value in both the parameter object
   * and the ROS system.
   */
  void set_bool(std::string param_name, bool value);

  /**
   * This function sets a previously declared parameter to a new value in both the parameter object
   * and the ROS system.
   */
  void set_int(std::string param_name, int64_t value);

  /**
   * This function sets a previously declared parameter to a new value in both the parameter object
   * and the ROS system.
   */
  void set_string(std::string param_name, std::string value);

  /**
   * This function should be called in the parametersCallback function in a containing ROS node.
   * It takes in a vector of changed parameters and updates them within the params_ object.
   * 
   * @param parameters: Vector of ROS Parameter objects that have been changed. 
   * @returns Boolean value corresponding to success or failure of the parameter changes
  */
  bool set_parameters_callback(const std::vector<rclcpp::Parameter> & parameters);

private:
  /**
   * Data structure to hold all of the parameters
  */
  std::map<std::string, std::variant<double, bool, int64_t, std::string>> params_;
  rclcpp::Node * container_node_;
};

} // namespace rosplane
#endif // PARAM_MANAGER_H
