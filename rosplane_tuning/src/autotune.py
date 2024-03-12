#!/usr/bin/env python3

from rosplane_msgs.msg import ControllerCommands
from rosplane_msgs.msg import ControllerInternalsDebug
from rosplane_msgs.msg import State
from std_srvs.srv import Trigger

import rclpy
from rclpy.node import Node

import numpy as np


class Autotune(Node):
    """
    This class is an auto-tuning node for the ROSplane autopilot. It calculates the error between
    the state estimate of the system response and the commanded setpoint for an autopilot. A
    gradient-based optimization is then run to find the optimal gains to minimize the error.

    Va is airspeed, phi is roll angle, chi is course angle, theta is pitch angle, and h is altitude.
    """

    def __init__(self):
        super().__init__('autotune')

        # Class variables
        self.collecting_data = False

        self.state = []
        self.commands = []
        self.internals_debug = []

        # ROS parameters
        # The amount of time to collect data for calculating the error
        self.declare_parameter('error_collection_period', rclpy.Parameter.Type.DOUBLE)
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
        self.error_collection_timer = self.create_timer(
            self.get_parameter('error_collection_period').value,
            self.error_collection_callback)
        self.error_collection_timer.cancel()

        # Services
        self.run_tuning_iteration_service = self.create_service(
            Trigger,
            'run_tuning_iteration',
            self.run_tuning_iteration_callback)


    ## ROS Callbacks ##
    def state_callback(self, msg):
        """
        This function is called when a new state estimate is received. It stores the state estimate
        if the node is currently collecting data.
        """

        if self.collecting_data:
            time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.state.append([time, msg.va, msg.phi, msg.chi, msg.theta, -msg.position[2]])  # h = -msg.position[2]

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

    def error_collection_callback(self):
        """
        This function is called when the error collection timer expires. It stops the error
        collection process and begins the optimization process.
        """

        self.stop_error_collection()

    def run_tuning_iteration_callback(self, request, response):
        """
        This function is called when the run_tuning_iteration service is called. It steps the autopilot
        command with the signal_generator node and begins the error collection process.
        """

        self.get_logger().info('Starting tuning iteration...')

        self.start_error_collection()

        response.success = True
        response.message = 'Tuning iteration started!'

        return response


    ## Helper Functions ##
    def start_error_collection(self):
        """
        Start the error collection timer and begin storing state and command data for the time specified
        by the error_collection_period parameter.
        """

        # Start data collection
        self.collecting_data = True

        # Start the timer
        self.error_collection_timer.timer_period_ns = \
                int(self.get_parameter('error_collection_period').value * 1e9)
        self.error_collection_timer.reset()

    def stop_error_collection(self):
        """
        Stop the error collection timer and stop storing state and command data.
        """

        self.collecting_data = False
        self.error_collection_timer.cancel()


def main(args=None):
    rclpy.init(args=args)

    autotune = Autotune()
    rclpy.spin(autotune)

    autotune.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

