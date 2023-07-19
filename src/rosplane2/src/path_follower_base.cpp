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


  update_timer_ = this->create_wall_timer(100ms, std::bind(&path_follower_base::update, this)); // TODO change this duration to change based on update rate.
  controller_commands_pub_  = this->create_publisher<rosplane2_msgs::msg::ControllerCommands>("controller_commands",1);

  this->declare_parameter("CHI_INFTY", params_.chi_infty);
  this->declare_parameter("K_PATH", params_.k_path);
  this->declare_parameter("K_ORBIT", params_.k_orbit);

  params_.chi_infty = this->get_parameter("CHI_INFTY").as_double();
  params_.k_path = this->get_parameter("K_PATH").as_double();
  params_.k_orbit = this->get_parameter("K_ORBIT").as_double();

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


    RCLCPP_DEBUG_STREAM(this->get_logger(), "Publishing Contoller Commands!");

    RCLCPP_DEBUG_STREAM(this->get_logger(), "chi_c: " << msg.chi_c);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "va_c: " << msg.va_c);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "h_c: " << msg.h_c);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "phi_ff: " << msg.phi_ff);

    RCLCPP_DEBUG_STREAM(this->get_logger(), "k_orbit: " << params_.k_orbit);



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

  RCLCPP_DEBUG_STREAM(this->get_logger(), "FROM STATE -- input.chi: " << input_.chi);


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

    rcl_interfaces::msg::SetParametersResult path_follower_base::parametersCallback(const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "";

        for (const auto &param : parameters) {

            if (param.get_name() == "CHI_INFTY"){
                params_.chi_infty = param.as_double();
                result.successful = true;
                result.reason = "success";
            }
            else if (param.get_name() == "K_PATH"){
                params_.k_path = param.as_double();
                result.successful = true;
                result.reason = "success";
            }
            else if (param.get_name() == "K_ORBIT"){
                params_.k_orbit = param.as_double();
                result.successful = true;
                result.reason = "success";
            }

        }

        return result;
    }

} //end namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<rosplane2::path_follower_example>());
  return 0;
}
