#ifndef PATH_FOLLOWER_BASE_H
#define PATH_FOLLOWER_BASE_H

#include <rclcpp/rclcpp.hpp>
#include <rosplane_msgs/msg/controller_commands.hpp>
#include <rosplane_msgs/msg/current_path.hpp>
#include <rosplane_msgs/msg/state.hpp>
#include <param_manager.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace rosplane
{

enum class path_type
{
  Orbit,
  Line
};

class path_follower_base : public rclcpp::Node
{
public:
  path_follower_base();
  float spin();

protected:
  struct input_s
  {
    enum path_type p_type;
    float Va_d;
    float r_path[3];
    float q_path[3];
    float c_orbit[3];
    float rho_orbit;
    int lam_orbit;
    float pn;  /** position north */
    float pe;  /** position east */
    float h;   /** altitude */
    float Va;  /** airspeed */
    float chi; /** course angle */
    float psi; /** heading angle */
  };

  struct output_s
  {
    double Va_c;   /** commanded airspeed (m/s) */
    double h_c;    /** commanded altitude (m) */
    double chi_c;  /** commanded course (rad) */
    double phi_ff; /** feed forward term for orbits (rad) */
  };

  // struct params_s
  // {
  //   double chi_infty;
  //   double k_path;
  //   double k_orbit;
  // };

  virtual void follow(const struct input_s & input,
                      struct output_s & output) = 0;
  // struct params_s params_ = {.5, 0.05, 4.0}; /**< params */
  param_manager params;

private:
  rclcpp::Subscription<rosplane_msgs::msg::State>::SharedPtr vehicle_state_sub_;
  rclcpp::Subscription<rosplane_msgs::msg::CurrentPath>::SharedPtr current_path_sub_;

  rclcpp::Publisher<rosplane_msgs::msg::ControllerCommands>::SharedPtr controller_commands_pub_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  double update_rate_ = 100.0;

  rosplane_msgs::msg::ControllerCommands controller_commands_;
  struct input_s input_;

  void vehicle_state_callback(const rosplane_msgs::msg::State::SharedPtr msg);
  bool state_init_;
  void current_path_callback(const rosplane_msgs::msg::CurrentPath::SharedPtr msg);
  bool current_path_init_;

  void update();

  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  rcl_interfaces::msg::SetParametersResult
  parametersCallback(const std::vector<rclcpp::Parameter> & parameters);

  /**
   * This declares each parameter as a parameter so that the ROS2 parameter system can recognize each parameter.
   * It also sets the default parameter, which can be overridden by a launch script.
   */
  void declare_parameters();
};

} // namespace rosplane

#endif // PATH_FOLLOWER_BASE_H
