    #include "controller_base.h"
    #include "controller_example.h"

    #include <iostream>

    namespace rosplane2
    {

    controller_base::controller_base() : Node("controller_base")
    {

        actuators_pub_ = this->create_publisher<rosplane2_msgs::msg::Command>("command",10); //Advertise
        internals_pub_ = this->create_publisher<rosplane2_msgs::msg::ControllerInternals>("controller_inners",10);

        timer_ = this->create_wall_timer(10ms, std::bind(&controller_base::actuator_controls_publish, this));

        controller_commands_sub_ = this->create_subscription<rosplane2_msgs::msg::ControllerCommands>(
                "controller_commands", 10, std::bind(&controller_base::controller_commands_callback, this, _1));
        vehicle_state_sub_ = this->create_subscription<rosplane2_msgs::msg::State>(
                "state", 10, std::bind(&controller_base::vehicle_state_callback, this, _1));

        command_recieved_ = false;

        declare_parameters();

        set_parameters();

        parameter_callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&controller_base::parametersCallback, this, std::placeholders::_1));


    }

    void controller_base::controller_commands_callback(const rosplane2_msgs::msg::ControllerCommands::SharedPtr msg)
    {
        command_recieved_ = true;
        controller_commands_ = *msg; // This callback had to be changed to no longer be a pointer.
    }

    void controller_base::vehicle_state_callback(const rosplane2_msgs::msg::State::SharedPtr msg)
    {
        vehicle_state_ = *msg;

//        std::cout << "Vehicle State received." << std::endl;
    }

    void controller_base::actuator_controls_publish()
    {
        struct input_s input;
        input.h =-vehicle_state_.position[2];
        input.va = vehicle_state_.va;
        input.phi = vehicle_state_.phi;
        input.theta = vehicle_state_.theta;
        input.chi = vehicle_state_.chi;
        input.p = vehicle_state_.p;
        input.q = vehicle_state_.q;
        input.r = vehicle_state_.r;
        input.Va_c = controller_commands_.va_c;
        input.h_c = controller_commands_.h_c;
        input.chi_c = controller_commands_.chi_c;
        input.phi_ff = controller_commands_.phi_ff;
        input.Ts = 0.01f;

        //  std::cout << "Controlling!" << std::endl;
        //
        //  std::cout << "command_recieved: " << command_recieved_ << std::endl;

        struct output_s output;
        if (command_recieved_ == true)
        {
        control(params_, input, output);

        convert_to_pwm(output);

        rosplane2_msgs::msg::Command actuators;
        /* publish actuator controls */

        rclcpp::Time now = this->get_clock()->now();
        actuators.header.stamp.sec = now.seconds();
        actuators.header.stamp.nanosec = now.nanoseconds();

        actuators.ignore = 0;
        actuators.mode = rosplane2_msgs::msg::Command::MODE_PASS_THROUGH;
        actuators.x = (std::isfinite(output.delta_a)) ? output.delta_a : 0.0f;
        actuators.y = output.delta_e;//(isfinite(output.delta_e)) ? output.delta_e : 0.0f;
        actuators.z = output.delta_r;//(isfinite(output.delta_r)) ? output.delta_r : 0.0f;
        actuators.f = output.delta_t;//(isfinite(output.delta_t)) ? output.delta_t : 0.0f;

        actuators_pub_->publish(actuators);

            if (internals_pub_->get_subscription_count() > 0)
            {
              rosplane2_msgs::msg::ControllerInternals inners;
              inners.phi_c = output.phi_c;
              inners.theta_c = output.theta_c;
              switch (output.current_zone)
              {
              case alt_zones::TAKE_OFF:
                inners.alt_zone = inners.ZONE_TAKE_OFF;
                break;
              case alt_zones::CLIMB:
                inners.alt_zone = inners.ZONE_CLIMB;
                break;
              case alt_zones::DESCEND:
                inners.alt_zone = inners.ZONE_DESEND;
                break;
              case alt_zones::ALTITUDE_HOLD:
                inners.alt_zone = inners.ZONE_ALTITUDE_HOLD;
                break;
              default:
                break;
              }
              inners.aux_valid = false;
              internals_pub_->publish(inners);
            }
        }
    }

    void controller_base::set_parameters() {
        params_.alt_hz = this->get_parameter("alt_hz").as_double();
        params_.alt_toz = this->get_parameter("alt_toz").as_double();
        params_.tau = this->get_parameter("tau").as_double();
        params_.c_kp = this->get_parameter("c_kp").as_double();
        params_.c_kd = this->get_parameter("c_kd").as_double();
        params_.c_ki = this->get_parameter("c_ki").as_double();
        params_.r_kp = this->get_parameter("r_kp").as_double();
        params_.r_kd = this->get_parameter("r_kd").as_double();
        params_.r_ki = this->get_parameter("r_ki").as_double();
        params_.p_kp = this->get_parameter("p_kp").as_double();
        params_.p_kd = this->get_parameter("p_kd").as_double();
        params_.p_ki = this->get_parameter("p_ki").as_double();
        params_.p_ff = this->get_parameter("p_ff").as_double();
        params_.a_p_kp = this->get_parameter("a_p_kp").as_double();
        params_.a_p_kd = this->get_parameter("a_p_kd").as_double();
        params_.a_p_ki = this->get_parameter("a_p_ki").as_double();
        params_.a_t_kp = this->get_parameter("a_t_kp").as_double();
        params_.a_t_kd = this->get_parameter("a_t_kd").as_double();
        params_.a_t_ki = this->get_parameter("a_t_ki").as_double();
        params_.a_kp = this->get_parameter("a_kp").as_double();
        params_.a_kd = this->get_parameter("a_kd").as_double();
        params_.a_ki = this->get_parameter("a_ki").as_double();
        params_.b_kp = this->get_parameter("b_kp").as_double();
        params_.b_kd = this->get_parameter("b_kd").as_double();
        params_.b_ki = this->get_parameter("b_ki").as_double();
        params_.trim_e = this->get_parameter("trim_e").as_double();
        params_.trim_a = this->get_parameter("trim_a").as_double();
        params_.trim_r = this->get_parameter("trim_r").as_double();
        params_.trim_t = this->get_parameter("trim_t").as_double();
        params_.max_e = this->get_parameter("max_e").as_double();
        params_.max_a = this->get_parameter("max_a").as_double();
        params_.max_r = this->get_parameter("max_r").as_double();
        params_.max_t = this->get_parameter("max_t").as_double();
        params_.pwm_rad_e = this->get_parameter("pwm_rad_e").as_double();
        params_.pwm_rad_a = this->get_parameter("pwm_rad_a").as_double();
        params_.pwm_rad_r = this->get_parameter("pwm_rad_r").as_double();
    }

    void controller_base::declare_parameters() {
        this->declare_parameter("alt_hz", params_.alt_hz);
        this->declare_parameter("alt_toz", params_.alt_toz);
        this->declare_parameter("tau", params_.tau);
        this->declare_parameter("c_kp", params_.c_kp);
        this->declare_parameter("c_kd", params_.c_kd);
        this->declare_parameter("c_ki", params_.c_ki);
        this->declare_parameter("r_kp", params_.r_kp);
        this->declare_parameter("r_kd", params_.r_kd);
        this->declare_parameter("r_ki", params_.r_ki);
        this->declare_parameter("p_kp", params_.p_kp);
        this->declare_parameter("p_kd", params_.p_kd);
        this->declare_parameter("p_ki", params_.p_ki);
        this->declare_parameter("p_ff", params_.p_ff);
        this->declare_parameter("a_p_kp", params_.a_p_kp);
        this->declare_parameter("a_p_kd", params_.a_p_kd);
        this->declare_parameter("a_p_ki", params_.a_p_ki);
        this->declare_parameter("a_t_kp", params_.a_t_kp);
        this->declare_parameter("a_t_kd", params_.a_t_kd);
        this->declare_parameter("a_t_ki", params_.a_t_ki);
        this->declare_parameter("a_kp", params_.a_kp);
        this->declare_parameter("a_kd", params_.a_kd);
        this->declare_parameter("a_ki", params_.a_ki);
        this->declare_parameter("b_kp", params_.b_kp);
        this->declare_parameter("b_kd", params_.b_kd);
        this->declare_parameter("b_ki", params_.b_ki);
        this->declare_parameter("trim_e", params_.trim_e);
        this->declare_parameter("trim_a", params_.trim_a);
        this->declare_parameter("trim_r", params_.trim_r);
        this->declare_parameter("trim_t", params_.trim_t);
        this->declare_parameter("max_e", params_.max_e);
        this->declare_parameter("max_a", params_.max_a);
        this->declare_parameter("max_r", params_.max_r);
        this->declare_parameter("max_t", params_.max_t);
        this->declare_parameter("pwm_rad_e", params_.pwm_rad_e);
        this->declare_parameter("pwm_rad_a", params_.pwm_rad_a);
        this->declare_parameter("pwm_rad_r", params_.pwm_rad_r);
    }

    rcl_interfaces::msg::SetParametersResult controller_base::parametersCallback(const std::vector<rclcpp::Parameter> &parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = false;
            result.reason = "";

            for (const auto &param : parameters) {

                if (param.get_name() == "alt_hz"){
                    params_.alt_hz = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "alt_toz"){
                    params_.alt_toz = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "tau"){
                    params_.tau = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "c_kp"){
                    params_.c_kp = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "c_kd"){
                    params_.c_kd = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "c_ki"){
                    params_.c_ki = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "r_kp"){
                    params_.r_kp = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "r_kd"){
                    params_.r_kd = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "r_ki"){
                    params_.r_ki = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "p_kp"){
                    params_.p_kp = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "p_kd"){
                    params_.p_kd = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "p_ki"){
                    params_.p_ki = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "p_ff"){
                    params_.p_ff = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "a_p_kp"){
                    params_.a_p_kp = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "a_p_kd"){
                    params_.a_p_kd = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "a_p_ki"){
                    params_.a_p_ki = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "a_t_kp"){
                    params_.a_t_kp = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "a_t_kd"){
                    params_.a_t_kd = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "a_t_ki"){
                    params_.a_t_ki = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "a_kp"){
                    params_.a_kp = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "a_kd"){
                    params_.a_kd = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "a_ki"){
                    params_.a_ki = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "b_kp"){
                    params_.b_kp = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "b_kd"){
                    params_.b_kd = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "b_ki"){
                    params_.b_ki = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "trim_e"){
                    params_.trim_e = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "trim_a"){
                    params_.trim_a = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "trim_r"){
                    params_.trim_r = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "trim_t"){
                    params_.trim_t = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "max_e"){
                    params_.max_e = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "max_a"){
                    params_.max_a = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "max_r"){
                    params_.max_r = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "max_t"){
                    params_.max_t = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "pwm_rad_e"){
                    params_.pwm_rad_e = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "pwm_rad_a"){
                    params_.pwm_rad_a = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }
                else if (param.get_name() == "pwm_rad_r"){
                    params_.pwm_rad_r = param.as_double();
                    result.successful = true;
                    result.reason = "success";
                }

            }

            return result;
        }

    void controller_base::convert_to_pwm(controller_base::output_s &output) {
            output.delta_e = output.delta_e*params_.pwm_rad_e;
            output.delta_a = output.delta_a*params_.pwm_rad_a;
            output.delta_r = output.delta_r*params_.pwm_rad_r;
        }

    } //end namespace

    int main(int argc, char **argv)
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<rosplane2::controller_example>());

        return 0;
    }
