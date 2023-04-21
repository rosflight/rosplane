#ifndef PATH_FOLLOWER_BASE_H
#define PATH_FOLLOWER_BASE_H

#include <rclcpp/rclcpp.hpp>
#include <rosplane2_msgs/msg/state.hpp>
#include <rosplane2_msgs/msg/controller_commands.hpp>
// #include <dynamic_reconfigure/server.h>
// #include <rosplane2_msgs/msg/controller_internals.hpp>
#include <rosplane2_msgs/msg/current_path.hpp>


namespace rosplane2
{

enum class path_type
{
  Orbit,
  Line
};

class path_follower_base
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

  // rclcpp::Publisher<rosplane2_msgs::msg::Command>::SharedPtr actuators_pub_;
  // rclcpp::Subscription<rosplane2_msgs::msg::ControllerCommands>::SharedPtr controller_commands_sub_;
  rclcpp::Subscription<rosplane2_msgs::msg::State>::SharedPtr vehicle_state_sub_;
  rclcpp::Subscription<rosplane2_msgs::msg::ControllerCommands>::SharedPtr current_path_sub_;

  rclcpp::Publisher<rosplane2_msgs::msg::ControllerCommands>::SharedPtr controller_commands_pub_;

  double update_rate_ = 100.0;
  // rclcpp::Timer update_timer_;
  rclcpp::WallTimer<CallbackT>::SharedPtr create_wall_timer(std::chrono::duration<DurationRepT, DurationT> period, CallbackT callback, rclcpp::CallbackGroup::SharedPtr group = nullptr)

  rosplane_msgs::msg::Controller_Commands controller_commands_;
  struct params_s  params_;            /**< params */
  struct input_s input_;

  void vehicle_state_callback(const rosplane_msgs::msg::StateConstPtr &msg);
  bool state_init_;
  void current_path_callback(const rosplane_msgs::msg::Current_PathConstPtr &msg);
  bool current_path_init_;

  dynamic_reconfigure::Server<rosplane::FollowerConfig> server_;
  dynamic_reconfigure::Server<rosplane::FollowerConfig>::CallbackType func_;
  // void reconfigure_callback(rosplane::FollowerConfig &config, uint32_t level);

  void update(const rclcpp::TimerEvent &);
};

} // end namespace

#endif // PATH_FOLLOWER_BASE_H
