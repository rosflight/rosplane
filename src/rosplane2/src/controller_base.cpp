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

        actuators.ignore = 0;
        actuators.mode = rosplane2_msgs::msg::Command::MODE_PASS_THROUGH;
        actuators.x = output.delta_a;//(isfinite(output.delta_a)) ? output.delta_a : 0.0f;
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

        void controller_base::convert_to_pwm(controller_base::output_s &output)
        {
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
