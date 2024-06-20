/**
 * @file param_manager.hpp
 * 
 * Manager for the parameters in ROSplane. Includes helper functions to call parameters
 * 
 * @author Jacob Moore <jacobmoor2@gmail.com>
*/

#ifndef PARAM_MANAGER_H
#define PARAM_MANAGER_H

#include <rclcpp/rclcpp.hpp>
#include <variant>

namespace rosplane
{
    
class param_manager
{
public:
  /**
   * Public constructor
   * 
   * @param node: the ROS2 node that has this parameter object. Used to poll the ROS2 parameters
  */
  param_manager(rclcpp::Node * node);

  /**
   * Helper functions to access parameter values stored in param_manager object
   * Returns a std::variant that holds the value of the given parameter
  */
  double get_double(std::string param_name);
  bool get_bool(std::string param_name);
  int64_t get_int(std::string param_name);
  std::string get_string(std::string param_name);

  /**
   * Helper functions to declare parameters in the param_manager object
   * Inserts a parameter into the parameter object and declares it with the ROS system
  */
  void declare_double(std::string param_name, double value);
  void declare_bool(std::string param_name, bool value);
  void declare_int(std::string param_name, int64_t value);  
  void declare_string(std::string param_name, std::string value);

  /**
   * This sets the parameters with the values in the params_ object from the supplied parameter file, or sets them to
   * the default if no value is given for a parameter.
   */
  // TODO: Check to make sure that setting a parameter before declaring it won't give an error.
  // Hypothesis is that it will break, but is that not desired behavior?
  void set_parameters();

  /**
   * This function sets a previously declared parameter to a new value in both the parameter object
   * and the ROS system.
   */
  void set_double(std::string param_name, double value);
  void set_bool(std::string param_name, bool value);
  void set_int(std::string param_name, int64_t value);
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


}   // namespace rosplane
#endif // PARAM_MANAGER_H