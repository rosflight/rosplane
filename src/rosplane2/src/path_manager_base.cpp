#include "path_manager_base.hpp"
#include "path_manager_example.hpp"
#include "rclcpp/rclcpp.hpp"
#include "iostream"


namespace rosplane2
{

path_manager_base::path_manager_base() : Node("rosplane2_path_manager")
{
  vehicle_state_sub_ = this->create_subscription<rosplane2_msgs::msg::State>("state", 10, std::bind(&path_manager_base::vehicle_state_callback,this,_1));
  new_waypoint_sub_  = this->create_subscription<rosplane2_msgs::msg::Waypoint>("waypoint_path", 10, std::bind(&path_manager_base::new_waypoint_callback,this,_1));
  current_path_pub_  = this->create_publisher<rosplane2_msgs::msg::CurrentPath>("current_path", 10);
  update_timer_      = this->create_wall_timer(10ms, std::bind(&path_manager_base::current_path_publish, this));
  // interesting read on wall timer
  // https://answers.ros.org/question/375561/create-wall-timer-using-callback-with-parameters-ros2-c/

  this->declare_parameter("R_min", 25.0);

  params_.R_min = this->get_parameter("R_min").as_double();

  num_waypoints_ = 0;

  state_init_ = false;
}

void path_manager_base::vehicle_state_callback(const rosplane2_msgs::msg::State &msg)
{

  vehicle_state_ = msg;

  state_init_ = true;
}

void path_manager_base::new_waypoint_callback(const rosplane2_msgs::msg::Waypoint &msg)
{
  if (msg.clear_wp_list == true)
  {
    waypoints_.clear();
    num_waypoints_ = 0;
    idx_a_ = 0;
    return;
  }
  if (msg.set_current || num_waypoints_ == 0)
  {
    waypoint_s currentwp;
    currentwp.w[0] = vehicle_state_.position[0];
    currentwp.w[1] = vehicle_state_.position[1];
    currentwp.w[2] = (vehicle_state_.position[2] > -25 ? msg.w[2] : vehicle_state_.position[2]);
    currentwp.chi_d = vehicle_state_.chi;
    currentwp.chi_valid = msg.chi_valid;

    currentwp.va_d = msg.va_d;

    waypoints_.clear();
    waypoints_.push_back(currentwp);
    num_waypoints_ = 1;
    idx_a_ = 0;
  }
  waypoint_s nextwp;
  nextwp.w[0]         = msg.w[0];
  nextwp.w[1]         = msg.w[1];
  nextwp.w[2]         = msg.w[2];
  nextwp.chi_d        = msg.chi_d;
  nextwp.chi_valid    = msg.chi_valid;
  nextwp.va_d         = msg.va_d;
  waypoints_.push_back(nextwp);
  num_waypoints_++;
}

 
void path_manager_base::current_path_publish() //const rclcpp::TimerEvent &
{

  struct input_s input;
  input.pn = vehicle_state_.position[0];               // position north
  input.pe = vehicle_state_.position[1];               // position east 
  input.h =  -vehicle_state_.position[2];              // altitude
  input.chi = vehicle_state_.chi;

  struct output_s output;

  if (state_init_ == true)
  {
    manage(params_, input, output);
  }

  rosplane2_msgs::msg::CurrentPath current_path;

  if (output.flag)
    current_path.path_type = current_path.LINE_PATH;
  else
    current_path.path_type = current_path.ORBIT_PATH;
  current_path.va_d = output.va_d;
  for (int i = 0; i < 3; i++)
  {
    current_path.r[i] = output.r[i];
    current_path.q[i] = output.q[i];
    current_path.c[i] = output.c[i];
  }
  current_path.rho = output.rho;
  current_path.lamda = output.lamda;

  RCLCPP_DEBUG_STREAM(this->get_logger(), "Publishing Current Path!");

  RCLCPP_DEBUG_STREAM(this->get_logger(), "Path Type: " << current_path.path_type);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "va_d: " << current_path.va_d);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "r: " << current_path.r[0] << ", " << current_path.r[1] << ", " << current_path.r[2]);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "q: " << current_path.q[0] << ", " << current_path.q[1] << ", " << current_path.q[2]);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "c: " << current_path.c[0] << ", " << current_path.c[1] << ", " << current_path.c[2]);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "rho: " << current_path.rho);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "lamda: " << current_path.lamda);


  current_path_pub_->publish(current_path);
} 


} //end namespace


int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rosplane2::path_manager_example>());

  return 0;
}