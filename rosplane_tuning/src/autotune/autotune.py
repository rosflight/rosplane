#!/usr/bin/env python3

from rosplane_msgs.msg import ControllerCommands
from rosplane_msgs.msg import ControllerInternalsDebug
from rosplane_msgs.msg import State
from std_srvs.srv import Trigger

import rclpy
from rclpy.node import Node

from optimizer import Optimizer


class Autotune(Node):
    """
    This class is an auto-tuning node for the ROSplane autopilot. The node calculates the error
    between the state estimate of the system and the commanded setpoint for a given autopilot.
    A gradient-based optimization is then run to find the optimal gains to minimize the error.

    This class itself contains the ROS-specific code for the autotune node. The optimization
    algorithms are contained in the Optimizer class.

    Va is airspeed, phi is roll angle, chi is course angle, theta is pitch angle, and h is altitude.
    """

    def __init__(self):
        super().__init__('autotune')

        # Class variables
        self.collecting_data = False
        self.state = []
        self.commands = []
        self.internals_debug = []
        self.new_gains = []  # TODO: Get gains from ROS parameters
        
        u1 = 10**-4     # 1st Strong Wolfe Condition, must be between 0 and 1.
        u2 = 0.5        # 2nd Strong Wolfe Condition, must be between u1 and 1.
        sigma = 1.5     
        alpha_init = 1  # Typically 1
        tau = 10**-3    # Convergence tolerance, typically 10^-3
        self.optimization_params = [u1,u2,sigma,alpha_init,tau]
        self.optimizer = Optimizer(self.new_gains, self.optimization_params)

        # ROS parameters
        # The amount of time to collect data for calculating the error
        self.declare_parameter('stabilize_period', rclpy.Parameter.Type.DOUBLE)
        # The autopilot that is currently being tuned
        self.declare_parameter('current_tuning_autopilot', rclpy.Parameter.Type.STRING)

        # Subscriptions
        self.state_subscription = self.create_subscription(
            State,
            'estimated_state',
            self.state_callback,
            10)
        self.commands_subscription = self.create_subscription(
            ControllerCommands,
            'controller_commands',
            self.commands_callback,
            10)
        self.internals_debug_subscription = self.create_subscription(
            ControllerInternalsDebug,
            'tuning_debug',
            self.internals_debug_callback,
            10)

        # Timers
        self.stabilize_period_timer = self.create_timer(
            self.get_parameter('stabilize_period').value,
            self.stabilize_period_timer_callback)
        self.stabilize_period_timer.cancel()

        # Services
        self.run_tuning_iteration_service = self.create_service(
            Trigger,
            'run_tuning_iteration',
            self.run_tuning_iteration_callback)

        # Clients
        self.toggle_step_signal_client = self.create_client(Trigger, 'toggle_step_signal')

        # Optimization Setup 
        # TODO: Implement this function
        # Run iteration at x0 -> phi0 
        # Run iteration at x0+0.01 -> temp_phi
        # Do a finite difference to get the gradient -> phi0_prime
        # Repeat for x0+alpha


    ## ROS Callbacks ##
    def state_callback(self, msg):
        """
        This function is called when a new state estimate is received. It stores the state estimate
        if the node is currently collecting data.
        """

        if self.collecting_data:
            time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.state.append([time, msg.va, msg.phi, msg.chi, msg.theta, 
                               -msg.position[2]])  # h = -msg.position[2]

    def commands_callback(self, msg):
        """
        This function is called when new commands are received. It stores the commands if the node
        is currently collecting data.
        """

        if self.collecting_data:
            time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.commands.append([time, msg.va_c, msg.chi_c, msg.h_c])

    def internals_debug_callback(self, msg):
        """
        This function is called when new debug information is received. It stores the debug
        information if the node is currently collecting data.
        """

        if self.collecting_data:
            time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.internals_debug.append([time, msg.phi_c, msg.theta_c])

    def stabilize_period_timer_callback(self):
        """
        This function is called when the stability timer callback occurs. It starts/stops data
        collection and sets up ROSplane to perform a step manuever.
        """

        if not self.collecting_data:
            # Stabilization period is over, start collecting data
            self.get_logger().info('Stepping command and collecting data for '
                                   + str(self.get_parameter('stabilize_period').value)
                                   + ' seconds...')
            self.collecting_data = True
            self.call_toggle_step_signal()
        else:
            # Data collection is over, stop collecting data and calculate gains for next iteration
            self.get_logger().info('Data collection complete.')
            self.collecting_data = False
            self.stabilize_period_timer.cancel()
            self.call_toggle_step_signal()
            self.new_gains = self.optimizer.get_next_parameter_set(self.calculate_error())


    def run_tuning_iteration_callback(self, request, response):
        """
        This function is called when the run_tuning_iteration service is called. It starts the
        next iteration of the optimization process.
        """

        if not self.optimizer.optimization_terminated():
            self.get_logger().info('Setting gains: ' + str(self.new_gains))
            self.set_gains(self.new_gains)

            self.stabilize_period_timer.timer_period_ns = \
                    int(self.get_parameter('stabilize_period').value * 1e9)
            self.stabilize_period_timer.reset()

            self.get_logger().info('Stabilizing autopilot for '
                                   + str(self.get_parameter('stabilize_period').value)
                                   + ' seconds...')

        response.success = True
        response.message = self.optimizer.get_optimiztion_status()

        return response


    ## Helper Functions ##
    def call_toggle_step_signal(self):
        """
        Call the signal_generator's toggle step service to toggle the step input.
        """

        while not self.toggle_step_signal_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {self.toggle_step_signal_client.srv_name} ' +
            'not available, waiting...')

        request = Trigger.Request()
        self.toggle_step_signal_client.call_async(request)

    def set_gains(self, gains):
        """
        Set the gains of the autopilot to the given values.
        """
        # TODO: Implement this function
        pass


def main(args=None):
    rclpy.init(args=args)

    autotune = Autotune()
    rclpy.spin(autotune)

    autotune.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

