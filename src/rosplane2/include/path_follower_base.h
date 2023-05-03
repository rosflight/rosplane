#ifndef PATH_FOLLOWER_BASE_H
#define PATH_FOLLOWER_BASE_H

#include <rclcpp/rclcpp.hpp>
#include <rosplane2_msgs/msg/state.hpp>
#include <rosplane2_msgs/msg/controller_commands.hpp>

// #include <rosplane2_msgs/msg/controller_internals.hpp>
#include <rosplane2_msgs/msg/current_path.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace rosplane2
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
    float pn;               /** position north */
    float pe;               /** position east */
    float h;                /** altitude */
    float Va;               /** airspeed */
    float chi;              /** course angle */
  };

  struct output_s
  {
    double Va_c;             /** commanded airspeed (m/s) */
    double h_c;              /** commanded altitude (m) */
    double chi_c;            /** commanded course (rad) */
    double phi_ff;           /** feed forward term for orbits (rad) */
  };

  struct params_s
  {
    double chi_infty;
    double k_path;
    double k_orbit;
  };

  virtual void follow(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

private:

  rclcpp::Subscription<rosplane2_msgs::msg::State>::SharedPtr vehicle_state_sub_;
  rclcpp::Subscription<rosplane2_msgs::msg::CurrentPath>::SharedPtr current_path_sub_;

  rclcpp::Publisher<rosplane2_msgs::msg::ControllerCommands>::SharedPtr controller_commands_pub_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  double update_rate_ = 100.0;

  rosplane2_msgs::msg::ControllerCommands controller_commands_;
  struct params_s  params_ = {1.0472, 0.025, 4.0};            /**< params */
  struct input_s input_;

  void vehicle_state_callback(const rosplane2_msgs::msg::State::SharedPtr msg);
  bool state_init_;
  void current_path_callback(const rosplane2_msgs::msg::CurrentPath::SharedPtr msg);
  bool current_path_init_;



  void update();

  rcl_interfaces::msg::SetParametersResult parametersCallback(
            const std::vector<rclcpp::Parameter> &parameters);
};

} // end namespace

#endif // PATH_FOLLOWER_BASE_H
