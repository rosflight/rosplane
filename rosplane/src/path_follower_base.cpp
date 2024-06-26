#include "path_follower_base.hpp"
#include "path_follower_example.hpp"
#include <rclcpp/logging.hpp>

namespace rosplane
{

path_follower_base::path_follower_base()
    : Node("path_follower_base"), params(this), params_initialized_(false)
{
  vehicle_state_sub_ = this->create_subscription<rosplane_msgs::msg::State>(
    "estimated_state", 10, std::bind(&path_follower_base::vehicle_state_callback, this, _1));
  current_path_sub_ = this->create_subscription<rosplane_msgs::msg::CurrentPath>(
    "current_path", 100,
    std::bind(&path_follower_base::current_path_callback, this, _1)); // the 1 may need to be 100

  controller_commands_pub_ =
    this->create_publisher<rosplane_msgs::msg::ControllerCommands>("controller_command", 1);

  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&path_follower_base::parametersCallback, this, std::placeholders::_1));

  // Declare and set parameters with the ROS2 system
  declare_parameters();
  params.set_parameters();

  params_initialized_ = true;

  // Now that the parameters have been set and loaded from the launch file, create the timer.
  set_timer();

  state_init_ = false;
  current_path_init_ = false;
}

void path_follower_base::set_timer() {
  double frequency = params.get_double("controller_commands_pub_frequency"); 
  timer_period_ = std::chrono::microseconds(static_cast<long long>(1.0 / frequency * 1e6));

  update_timer_ = this->create_wall_timer(
    timer_period_,
    std::bind(&path_follower_base::update,
              this));
}

void path_follower_base::update()
{

  struct output_s output;

  if (state_init_ == true && current_path_init_ == true) {
    follow(input_, output);
    rosplane_msgs::msg::ControllerCommands msg;

    rclcpp::Time now = this->get_clock()->now();

    msg.header.stamp = now;

    msg.chi_c = output.chi_c;
    msg.va_c = output.va_c;
    msg.h_c = output.h_c;
    msg.phi_ff = output.phi_ff;

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Publishing Contoller Commands!");

    RCLCPP_DEBUG_STREAM(this->get_logger(), "chi_c: " << msg.chi_c);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "va_c: " << msg.va_c);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "h_c: " << msg.h_c);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "phi_ff: " << msg.phi_ff);

    // RCLCPP_DEBUG_STREAM(this->get_logger(), "k_orbit: " << params.get_double("k_orbit"));

    controller_commands_pub_->publish(msg);
  }
}

void path_follower_base::vehicle_state_callback(const rosplane_msgs::msg::State::SharedPtr msg)
{
  input_.pn = msg->position[0]; /** position north */
  input_.pe = msg->position[1]; /** position east */
  input_.h = -msg->position[2]; /** altitude */
  input_.chi = msg->chi;
  input_.psi = msg->psi;
  input_.va = msg->va;

  RCLCPP_DEBUG_STREAM(this->get_logger(), "FROM STATE -- input.chi: " << input_.chi);

  state_init_ = true;
}

void path_follower_base::current_path_callback(const rosplane_msgs::msg::CurrentPath::SharedPtr msg)
{
  if (msg->path_type == msg->LINE_PATH) input_.p_type = path_type::Line;
  else if (msg->path_type == msg->ORBIT_PATH)
    input_.p_type = path_type::Orbit;

  input_.va_d = msg->va_d;
  for (int i = 0; i < 3; i++) {
    input_.r_path[i] = msg->r[i];
    input_.q_path[i] = msg->q[i];
    input_.c_orbit[i] = msg->c[i];
  }
  input_.rho_orbit = msg->rho;
  input_.lam_orbit = msg->lamda;
  current_path_init_ = true;
}

rcl_interfaces::msg::SetParametersResult
path_follower_base::parametersCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "One of the parameters given is not a parameter of the path_follower node";

  bool success = params.set_parameters_callback(parameters);
  if (success)
  {
    result.successful = true;
    result.reason = "success";
  }

  if (params_initialized_ && success) {
    double frequency = params.get_double("controller_commands_pub_frequency");

    std::chrono::microseconds curr_period = std::chrono::microseconds(static_cast<long long>(1.0 / frequency * 1e6));
    if (timer_period_ != curr_period) {
      update_timer_->cancel();
      set_timer();
    }
  }

  return result;
}

void path_follower_base::declare_parameters()
{
  params.declare_double("controller_commands_pub_frequency", 10.0);
  params.declare_double("chi_infty", .5);
  params.declare_double("k_path", 0.05);
  params.declare_double("k_orbit", 4.0);
  params.declare_int("update_rate", 100);
  params.declare_double("gravity", 9.81);
}

} // namespace rosplane

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<rosplane::path_follower_example>());
  return 0;
}
