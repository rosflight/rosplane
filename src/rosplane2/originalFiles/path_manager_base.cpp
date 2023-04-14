#include "path_manager_base.hpp"
#include "path_manager_example.hpp"

namespace rosplane2
{



path_manager_base::path_manager_base() 
//path_manager_base::path_manager_base: public rclcpp::Node
  //nh_(rclcpp::Node()),
  //nh_private_(rclcpp::Node("~"))
  
/* class path_manager_base::path_manager_base()// : public rclcpp::Node
{
  auto nh_ = rclcpp::Node::make_shared("talker");

  public:
  nh_():Node("min_pub"),count_(0)
  {
    pub = this->create_publisher<std_msgs::msg::String>("topic",10);
    timer_ = this-> create_wall_timer(500ms, std::bind(&min_pub::timer_callback, this));
  }

  //auto nh_ = rclcpp::Node::make_shared("talker");
  //auto nh_private_ = 

} */

{
  // python
  //rclcpp::Node::SharedPtr vehicle_state_sub_ = create_subscription(rosplane2_msgs::msg::State, 'state', &path_manager_base::vehicle_state_callback, 10)
  //rclcpp::Node::SharedPtr new_waypoint_sub_ = create_subscription(rosplane2_msgs::msg::Waypoint, 'waypoint_path', &path_manager_base::new_waypoint_callback, 10)
  //rclcpp::Node::SharedPtr current_path_pub_ = create_publisher(rosplane2_msgs::msg::CurrentPath, 'current_path', 10)
  using std::placeholders::_1;
  auto vehicle_state_sub_ = this->create_subscription<rosplane2_msgs::msg::State>("state", 10, std::bind(&path_manager_base::vehicle_state_callback,this,_1));
  auto new_waypoint_sub_  = this->create_subscription<rosplane2_msgs::msg::Waypoint>("waypoint_path", 10, std::bind(&path_manager_base::new_waypoint_callback,this,_1));
  auto current_path_pub_  = this->create_publisher<rosplane2_msgs::msg::CurrentPath>("current_path", 10);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(vehicle_state_sub_);
  executor.add_node(new_waypoint_sub_);
  executor.add_node(current_path_pub_);


  //nh_private_.set_parameter_if_not_set("R_min", params_.R_min, 25.0);
  //nh_private_.set_parameter_if_not_set("update_rate", update_rate_, 10.0);

  // vehicle_state_sub_ = nh_.subscribe("state", 10, &path_manager_base::vehicle_state_callback, this);
  // new_waypoint_sub_ = nh_.subscribe("waypoint_path", 10, &path_manager_base::new_waypoint_callback, this);

  //vehicle_state_sub_ = nh_.create_subscription("state", 10, &path_manager_base::vehicle_state_callback, this);
  //new_waypoint_sub_ = nh_.create_subscription("waypoint_path", 10, &path_manager_base::new_waypoint_callback, this);

  //auto node = rclcpp::Node::make_shared("talker")
  //auto chatter_pub = node->create_publisher<std_msgs::msg::String>("chatter",1000);
  rclcpp::Rate loop_rate(10)
  //vehicle_state_sub_ = this -> create_subscription<std_msgs::msg::String>("state",10);
  //vehicle_state_sub_ = this -> create_subscription<rosplane2::msg::State.msg>("state",10);
  //new_waypoint_sub_ = nh_.subscribe("waypoint_path", 10, &path_manager_base::new_waypoint_callback, this);

  //current_path_pub_ = nh_.advertise<rosplane2_msgs::msg::CurrentPath>("current_path", 10);

  update_timer_ = nh_.createTimer(rclcpp::Duration(1.0/update_rate_), &path_manager_base::current_path_publish, this);

  num_waypoints_ = 0;

  state_init_ = false;
}

void path_manager_base::vehicle_state_callback(const rosplane2_msgs::StateConstPtr &msg)
{
  vehicle_state_ = *msg;

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

void path_manager_base::current_path_publish(const rclcpp::TimerEvent &)
{

  struct input_s input;
  input.pn = vehicle_state_.position[0];               /** position north */
  input.pe = vehicle_state_.position[1];               /** position east */
  input.h =  -vehicle_state_.position[2];                /** altitude */
  input.chi = vehicle_state_.chi;

  struct output_s output;

  if (state_init_ == true)
  {
    manage(params_, input, output);
  }

  rosplane2_msgs::msg::CurrentPath current_path;

  if (output.flag)
    rcpputils::fs::current_path.path_type = current_path.LINE_PATH;
  else
    rcpputils::fs::current_path.path_type = current_path.ORBIT_PATH;
  rcpputils::fs::current_path.va_d = output.va_d;
  for (int i = 0; i < 3; i++)
  {
    current_path.r[i] = output.r[i];
    current_path.q[i] = output.q[i];
    current_path.c[i] = output.c[i];
  }
  current_path.rho = output.rho;
  current_path.lamda = output.lamda;

  current_path_publish.publish(current_path);
}

} //end namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv, "rosplane2_path_manager");
  rosplane2::path_manager_base *est = new rosplane2::path_manager_example();


  executor.spin();

  //rclcpp::spin();

  return 0;
}
