#include "path_follower_base.h"
#include "path_follower_example.h"

namespace rosplane2
{

path_follower_base::path_follower_base(): Node("path_follower_base")
{
  vehicle_state_sub_ = this->create_subscription<rosplane2_msgs::msg::State>(
                "state", 10, std::bind(&path_follower_base::vehicle_state_callback, this, _1));
  current_path_sub_ = this->create_subscription<rosplane2_msgs::msg::CurrentPath>(
                "current_path", 100, std::bind(&path_follower_base::current_path_callback, this, _1)); // the 1 may need to be 100
  /*controller_commands_sub_ = this->create_subscription<rosplane2_msgs::msg::ControllerCommands>(
          "controller_commands", 10, std::bind(&controller_base::controller_commands_callback, this, _1));
  vehicle_state_sub_ = this->create_subscription<rosplane2_msgs::msg::State>(
          "state", 10, std::bind(&controller_base::vehicle_state_callback, this, _1));*/
  // vehicle_state_sub_ = nh_.subscribe<rosplane_msgs::msg::State>("state", 1, &path_follower_base::vehicle_state_callback, this);
  // current_path_sub_ = nh_.subscribe<rosplane_msgs::msg::Current_Path>("current_path", 1,
  //                     &path_follower_base::current_path_callback, this);


  // nh_private_.param<double>("CHI_INFTY", params_.chi_infty, 1.0472);
  // nh_private_.param<double>("K_PATH", params_.k_path, 0.025);
  // nh_private_.param<double>("K_ORBIT", params_.k_orbit, 4.0);

  // auto func_ = std::bind(&path_follower_base::reconfigure_callback, this, std::placeholders::_1, std::placeholders::_2); /// these may need to be enabled after all; they depend on a config file that doens't exist.
  // server_.setCallback(func_);
  update_timer_ = this->create_wall_timer(100ms, std::bind(&path_follower_base::update, this)); /// 100ms?
  // update_timer_ = nh_.createTimer(rclcpp::Duration(1.0/update_rate_), &path_follower_base::update, this);
  // controller_commands_pub_ = nh_.advertise<rosplane_msgs::msg::Controller_Commands>("controller_commands", 1);
  controller_commands_pub_  = this->create_publisher<rosplane2_msgs::msg::ControllerCommands>("controller_commands",1);

  state_init_ = false;
  current_path_init_ = false;
}

void path_follower_base::update()
{

  struct output_s output;

  if (state_init_ == true && current_path_init_ == true)
  {
    follow(params_, input_, output);
    rosplane2_msgs::msg::ControllerCommands msg;
    msg.chi_c = output.chi_c;
    msg.va_c = output.Va_c;
    msg.h_c = output.h_c;
    msg.phi_ff = output.phi_ff;
    controller_commands_pub_->publish(msg);
  }
}

void path_follower_base::vehicle_state_callback(const rosplane2_msgs::msg::State::SharedPtr msg)
{
  input_.pn = msg->position[0];               /** position north */
  input_.pe = msg->position[1];               /** position east */
  input_.h = -msg->position[2];                /** altitude */
  input_.chi = msg->chi;
  input_.Va = msg->va;

  state_init_ = true;
}

void path_follower_base::current_path_callback(const rosplane2_msgs::msg::CurrentPath::SharedPtr msg)
{
  if (msg->path_type == msg->LINE_PATH)
    input_.p_type = path_type::Line;
  else if (msg->path_type == msg->ORBIT_PATH)
    input_.p_type = path_type::Orbit;

  input_.Va_d = msg->va_d;
  for (int i = 0; i < 3; i++)
  {
    input_.r_path[i] = msg->r[i];
    input_.q_path[i] = msg->q[i];
    input_.c_orbit[i] = msg->c[i];
  }
  input_.rho_orbit = msg->rho;
  input_.lam_orbit = msg->lamda;
  current_path_init_ = true;
}

// no longer used for compatibility reasons
/*void path_follower_base::reconfigure_callback(rosplane::FollowerConfig &config, uint32_t level)
{
  params_.chi_infty = config.CHI_INFTY;
  params_.k_path = config.K_PATH;
  params_.k_orbit = config.K_ORBIT;
}*/
} //end namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);//, "rosplane_path_follower");
  // rosplane2::path_follower_base *path = new rosplane2::path_follower_example();

  rclcpp::spin(std::make_shared<rosplane2::path_follower_example>());
  // rclcpp::spin(path);

  return 0;
}
