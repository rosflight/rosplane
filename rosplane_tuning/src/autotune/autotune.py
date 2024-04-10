#!/usr/bin/env python3

from rosplane_msgs.msg import ControllerCommands
from rosplane_msgs.msg import ControllerInternalsDebug
from rosplane_msgs.msg import State
from optimizer import Optimizer

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from std_srvs.srv import Trigger

from enum import Enum, auto
from queue import Queue
import numpy as np
import time


class CurrentAutopilot(Enum):
    """
    This class defines which autopilots are available for tuning.
    """
    ROLL = auto()
    COURSE = auto()
    PITCH = auto()
    ALTITUDE = auto()
    AIRSPEED = auto()


class AutoTuneState(Enum):
    """
    This class defines the possible states of the autotune node.
    """
    ORBITING = auto()
    STABILIZING = auto()
    STEP_TESTING = auto()
    RETURN_TESTING = auto()


class Autotune(Node):
    """
    This class is an auto-tuning node for the ROSplane autopilot. The node calculates the error
    between the state estimate of the system and the commanded setpoint for a given autopilot.
    A gradient-based optimization is then run to find the optimal gains to minimize the error.

    This class itself contains the ROS-specific code for the autotune node. The optimization
    algorithms are contained in the Optimizer class. The core of the autotune procedure and logic
    is in the run_tuning_iteration_callback function and the stabilize_period_timer_callback
    function.

    Va is airspeed, phi is roll angle, chi is course angle, theta is pitch angle, and h is altitude.
    """

    def __init__(self):
        super().__init__('autotune')

        self.autotune_state = AutoTuneState.ORBITING

        # Callback groups, used for allowing external services to run mid-internal callback
        self.internal_callback_group = MutuallyExclusiveCallbackGroup()
        self.external_callback_group = MutuallyExclusiveCallbackGroup()

        # Class state variables
        self.collecting_data = False
        self.gain_queue = Queue()
        self.error_queue = Queue()

        # Data storage
        self.state = []
        self.commands = []
        self.internals_debug = []

        # ROS parameters
        # The amount of time to collect data for calculating the error
        self.declare_parameter('stabilize_period', rclpy.Parameter.Type.DOUBLE)
        # Whether to run the autotune continuously or not
        self.declare_parameter('continuous_tuning', rclpy.Parameter.Type.BOOL)
        # The autopilot that is currently being tuned
        self.declare_parameter('current_tuning_autopilot', rclpy.Parameter.Type.STRING)
        # Get the autopilot to tune
        if self.get_parameter('current_tuning_autopilot').value == 'roll':
            self.current_autopilot = CurrentAutopilot.ROLL
        elif self.get_parameter('current_tuning_autopilot').value == 'course':
            self.current_autopilot = CurrentAutopilot.COURSE
        elif self.get_parameter('current_tuning_autopilot').value == 'pitch':
            self.current_autopilot = CurrentAutopilot.PITCH
        elif self.get_parameter('current_tuning_autopilot').value == 'altitude':
            self.current_autopilot = CurrentAutopilot.ALTITUDE
        elif self.get_parameter('current_tuning_autopilot').value == 'airspeed':
            self.current_autopilot = CurrentAutopilot.AIRSPEED
        else:
            raise ValueError(self.get_parameter('current_tuning_autopilot').value +
                             ' is not a valid value for current_tuning_autopilot.' +
                             ' Please select one of the' +
                             ' following: roll, course, pitch, altitude, airspeed.')

        # Subscriptions
        self.state_subscription = self.create_subscription(
            State,
            'estimated_state',
            self.state_callback,
            10,
            callback_group=self.internal_callback_group)
        self.commands_subscription = self.create_subscription(
            ControllerCommands,
            'controller_commands',
            self.commands_callback,
            10,
            callback_group=self.internal_callback_group)
        self.internals_debug_subscription = self.create_subscription(
            ControllerInternalsDebug,
            'tuning_debug',
            self.internals_debug_callback,
            10,
            callback_group=self.internal_callback_group)

        # Timers
        self.stabilize_period_timer = self.create_timer(
            self.get_parameter('stabilize_period').value,
            self.stabilize_period_timer_callback,
            callback_group=self.internal_callback_group)
        self.stabilize_period_timer.cancel()

        # Services
        self.run_tuning_iteration_service = self.create_service(
            Trigger,
            'run_tuning_iteration',
            self.run_tuning_iteration_callback,
            callback_group=self.internal_callback_group)

        # Clients
        self.toggle_step_signal_client = self.create_client(
                Trigger,
                'toggle_step_signal',
                callback_group=self.external_callback_group)
        self.get_parameter_client = self.create_client(
                GetParameters,
                '/autopilot/get_parameters',
                callback_group=self.external_callback_group)
        self.autopilot_set_param_client = self.create_client(
                SetParameters,
                '/autopilot/set_parameters',
                callback_group=self.external_callback_group)
        self.signal_generator_set_param_client = self.create_client(
                SetParameters,
                '/signal_generator/set_parameters',
                callback_group=self.external_callback_group)

        # Optimization
        self.initial_gains = None  # get_gains function cannot be called in __init__ since the node
                                   # has not yet been passed to the executor

        # As we conduct optimizations, these parameters will be changed to be more
        # efficient to optimize the specific gains, based on the design spaces.
        if self.current_autopilot == CurrentAutopilot.ROLL:
            self.optimization_params = {'mu_1': 1e-3,
                                        'mu_2': 0.5,
                                        'sigma': 2.0,
                                        'alpha': 0.05,
                                        'tau': 1e-2,
                                        'h': 1e-2}
        elif self.current_autopilot == CurrentAutopilot.PITCH:
            self.optimization_params = {'mu_1': 1e-3,
                                        'mu_2': 0.5,
                                        'sigma': 2.0,
                                        'alpha': 0.05,
                                        'tau': 1e-2,
                                        'h': 1e-2}
        elif self.current_autopilot == CurrentAutopilot.COURSE:
            self.optimization_params = {'mu_1': 1e-3,
                                        'mu_2': 0.5,
                                        'sigma': 2.0,
                                        'alpha': 0.05,
                                        'tau': 1e-2,
                                        'h': 1e-2}
        elif self.current_autopilot == CurrentAutopilot.ALTITUDE:
            self.optimization_params = {'mu_1': 1e-3,
                                        'mu_2': 0.5,
                                        'sigma': 2.0,
                                        'alpha': 0.05,
                                        'tau': 1e-2,
                                        'h': 1e-2}
        elif self.current_autopilot == CurrentAutopilot.AIRSPEED:
            self.optimization_params = {'mu_1': 1e-3,
                                        'mu_2': 0.5,
                                        'sigma': 2.0,
                                        'alpha': 0.05,
                                        'tau': 1e-2,
                                        'h': 1e-2}
        else:
            raise ValueError(self.get_parameter('current_tuning_autopilot').value +
                             ' is not a valid value for current_tuning_autopilot.' +
                             ' Please select one of the' +
                             ' following: roll, course, pitch, altitude, airspeed.')

        self.optimizer = None


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

        if self.autotune_state == AutoTuneState.STABILIZING:
            # Stabilization period is over, start collecting data
            self.get_logger().info('Setting gains: ' + str(self.new_gains))
            self.set_current_gains(self.new_gains)
            self.get_logger().info('Stepping command for '
                                   + str(self.get_parameter('stabilize_period').value)
                                   + ' seconds...')
            self.collecting_data = True
            self.call_toggle_step_signal()
            self.autotune_state = AutoTuneState.STEP_TESTING

        elif self.autotune_state == AutoTuneState.STEP_TESTING:
            # Step test is over, reverse step
            self.get_logger().info('Reversing step command for '
                                   + str(self.get_parameter('stabilize_period').value)
                                   + ' seconds...')
            self.call_toggle_step_signal()
            self.autotune_state = AutoTuneState.RETURN_TESTING

        else:
            # Data collection is over, stop collecting data and calculate gains for next iteration
            self.get_logger().info('Data collection complete, restoring original gains and ' +
                                   'returning to orbit.')
            self.collecting_data = False
            self.stabilize_period_timer.cancel()
            self.set_current_gains(self.initial_gains)
            self.disable_autopilot_override()
            self.autotune_state = AutoTuneState.ORBITING

            if self.get_parameter('continuous_tuning').value:
                self.run_tuning_iteration_callback(Trigger.Request(), Trigger.Response())

    def run_tuning_iteration_callback(self, request, response):
        """
        This function is called when the run_tuning_iteration service is called. It starts the
        next iteration of the optimization process.
        """

        if self.optimizer is None:
            # Initialize optimizer
            self.initial_gains = self.get_current_gains()
            self.optimizer = Optimizer(self.initial_gains, self.optimization_params)
            self.new_gains = self.initial_gains
            self.set_signal_generator_params()

        elif not self.optimizer.optimization_terminated():
            self.new_gains = self.get_next_gains()

        if not self.optimizer.optimization_terminated():
            self.stabilize_period_timer.timer_period_ns = \
                    int(self.get_parameter('stabilize_period').value * 1e9)
            self.stabilize_period_timer.reset()
            self.enable_autopilot_override()
            self.get_logger().info('Stabilizing autopilot for '
                                   + str(self.get_parameter('stabilize_period').value)
                                   + ' seconds...')
            self.autotune_state = AutoTuneState.STABILIZING
            self.get_logger().info(f"State: {self.optimizer.get_optimization_status()}")

        else:
            self.get_logger().info('Optimization terminated with: ' +
                                   self.optimizer.get_optimization_status())
            self.stabilize_period_timer.cancel()

        response.success = True
        response.message = self.optimizer.get_optimization_status()

        return response


    ## Helper Functions ##
    def set_signal_generator_params(self):
        """
        Sets the signal generators parameters to the initial values needed for the optimization.
        """

        request = SetParameters.Request()
        if self.current_autopilot == CurrentAutopilot.ROLL:
            request.parameters = [
                    Parameter(name='controller_output', value='roll').to_parameter_msg(),
                    Parameter(name='signal_type', value='step').to_parameter_msg(),
                    Parameter(name='signal_magnitude', value=np.deg2rad(30)).to_parameter_msg(),
                    Parameter(name='default_phi_c', value=0.0).to_parameter_msg()
                    ]
        elif self.current_autopilot == CurrentAutopilot.COURSE:
            request.parameters = [
                    Parameter(name='controller_output', value='course').to_parameter_msg(),
                    Parameter(name='signal_type', value='step').to_parameter_msg(),
                    Parameter(name='signal_magnitude', value=np.deg2rad(90)).to_parameter_msg(),
                    ]
        elif self.current_autopilot == CurrentAutopilot.PITCH:
            request.parameters = [
                    Parameter(name='controller_output', value='pitch').to_parameter_msg(),
                    Parameter(name='signal_type', value='step').to_parameter_msg(),
                    Parameter(name='signal_magnitude', value=np.deg2rad(20)).to_parameter_msg(),
                    Parameter(name='default_theta_c', value=0.0).to_parameter_msg()
                    ]
        elif self.current_autopilot == CurrentAutopilot.ALTITUDE:
            request.parameters = [
                    Parameter(name='controller_output', value='altitude').to_parameter_msg(),
                    Parameter(name='signal_type', value='step').to_parameter_msg(),
                    Parameter(name='signal_magnitude', value=10.0).to_parameter_msg(),
                    ]
        else:  # CurrentAutopilot.AIRSPEED
            request.parameters = [
                    Parameter(name='controller_output', value='airspeed').to_parameter_msg(),
                    Parameter(name='signal_type', value='step').to_parameter_msg(),
                    Parameter(name='signal_magnitude', value=5.0).to_parameter_msg(),
                    Parameter(name='default_theta_c', value=0.0).to_parameter_msg()
                    ]


        # Call the service
        while not self.signal_generator_set_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service {self.signal_generator_set_param_client.srv_name}' +
                                   ' not available, waiting...')
        future = self.signal_generator_set_param_client.call_async(request)

        # Wait for the service to complete, exiting if it takes too long
        call_time = time.time()
        callback_complete = False
        while call_time + 5 > time.time():
            if future.done():
                callback_complete = True
                break
        if not callback_complete or future.result() is None:
            raise RuntimeError('Unable to set signal generator gains.')

        # Print any errors that occurred
        for response in future.result().results:
            if not response.successful:
                raise RuntimeError(f'Failed to set parameter: {response.reason}')

    def enable_autopilot_override(self):
        """
        Enable to autopilot override for the current autopilot, if an override is needed.
        This enables direct control of that autopilot, rather than the outer loop controlling
        the autopilot.
        """
        request = SetParameters.Request()
        if self.current_autopilot == CurrentAutopilot.ROLL:
            request.parameters = [
                    Parameter(name='roll_tuning_debug_override', value=True).to_parameter_msg()
                    ]
        elif self.current_autopilot == CurrentAutopilot.PITCH \
                or self.current_autopilot == CurrentAutopilot.AIRSPEED:
            request.parameters = [
                    Parameter(name='pitch_tuning_debug_override', value=True).to_parameter_msg()
                    ]
        self.set_autopilot_params(request)

    def disable_autopilot_override(self):
        """
        Disable the autopilot override for the current autopilot, if an override is needed.
        This allows the outer loop to control the autopilot again.
        """
        request = SetParameters.Request()
        if self.current_autopilot == CurrentAutopilot.ROLL:
            request.parameters = [
                    Parameter(name='roll_tuning_debug_override', value=False).to_parameter_msg()
                    ]
        elif self.current_autopilot == CurrentAutopilot.PITCH \
                or self.current_autopilot == CurrentAutopilot.AIRSPEED:
            request.parameters = [
                    Parameter(name='pitch_tuning_debug_override', value=False).to_parameter_msg()
                    ]
        self.set_autopilot_params(request)

    def call_toggle_step_signal(self):
        """
        Call the signal_generator's toggle step service to toggle the step input.
        """

        while not self.toggle_step_signal_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {self.toggle_step_signal_client.srv_name} ' +
            'not available, waiting...')

        self.toggle_step_signal_client.call_async(Trigger.Request())

    def get_current_gains(self):
        """
        Gets the current gains of the autopilot.

        Returns:
        np.array, size num_gains, dtype float: The gains of the autopilot. [gain1, gain2, ...]
        """

        request = GetParameters.Request()
        if self.current_autopilot == CurrentAutopilot.ROLL:
            request.names = ['r_kp', 'r_kd']
        elif self.current_autopilot == CurrentAutopilot.COURSE:
            request.names = ['c_kp', 'c_ki']
        elif self.current_autopilot == CurrentAutopilot.PITCH:
            request.names = ['p_kp', 'p_kd']
        elif self.current_autopilot == CurrentAutopilot.ALTITUDE:
            request.names = ['a_kp', 'a_ki']
        else:  # CurrentAutopilot.AIRSPEED
            request.names = ['a_t_kp', 'a_t_ki']

        # Call the service
        while not self.get_parameter_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {self.get_parameter_client.srv_name}' +
                                   ' not available, waiting...')
        future = self.get_parameter_client.call_async(request)

        # Wait for the service to complete, exiting if it takes too long
        call_time = time.time()
        callback_complete = False
        while call_time + 5 > time.time():
            if future.done():
                callback_complete = True
                break
        if not callback_complete or future.result() is None:
            raise RuntimeError('Unable to get autopilot gains.')

        # Check that values were returned. No values are returned if the parameter does not exist.
        if len(future.result().values) == 0:
            raise RuntimeError(f'Parameter values for {request.names} were not returned, ' +
                               'have they been set?')

        # Put the gains into a numpy array
        gains = []
        for value in future.result().values:
            if value.type != ParameterType.PARAMETER_DOUBLE:
                raise RuntimeError('Parameter type returned by get_parameters is not DOUBLE.')
            gains.append(value.double_value)

        return np.array(gains)

    def set_current_gains(self, gains):
        """
        Set the gains of the autopilot to the given values.

        Parameters:
        gains (np.array, size num_gains, dtype float): The gains to set for the autopilot.
            [gain1, gain2, ...]
        """

        request = SetParameters.Request()
        if self.current_autopilot == CurrentAutopilot.ROLL:
            request.parameters = [Parameter(name='r_kp', value=gains[0]).to_parameter_msg(),
                                  Parameter(name='r_kd', value=gains[1]).to_parameter_msg()]
        elif self.current_autopilot == CurrentAutopilot.COURSE:
            request.parameters = [Parameter(name='c_kp', value=gains[0]).to_parameter_msg(),
                                  Parameter(name='c_ki', value=gains[1]).to_parameter_msg()]
        elif self.current_autopilot == CurrentAutopilot.PITCH:
            request.parameters = [Parameter(name='p_kp', value=gains[0]).to_parameter_msg(),
                                  Parameter(name='p_kd', value=gains[1]).to_parameter_msg()]
        elif self.current_autopilot == CurrentAutopilot.ALTITUDE:
            request.parameters = [Parameter(name='a_kp', value=gains[0]).to_parameter_msg(),
                                  Parameter(name='a_ki', value=gains[1]).to_parameter_msg()]
        else:  # CurrentAutopilot.AIRSPEED
            request.parameters = [Parameter(name='a_t_kp', value=gains[0]).to_parameter_msg(),
                                  Parameter(name='a_t_ki', value=gains[1]).to_parameter_msg()]

        self.set_autopilot_params(request)

    def set_autopilot_params(self, request):
        """
        Set parameters for the autopilot. Moved to a separate function to avoid code repetition.

        Parameters:
        request (SetParameters.Request): A ROS2 object for setting parameters, already populated
            with the desired parameters and values.
        """

        # Call the service
        while not self.autopilot_set_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service {self.set_parameter_client.srv_name}' +
                                   ' not available, waiting...')
        future = self.autopilot_set_param_client.call_async(request)

        # Wait for the service to complete, exiting if it takes too long
        call_time = time.time()
        callback_complete = False
        while call_time + 5 > time.time():
            if future.done():
                callback_complete = True
                break
        if not callback_complete or future.result() is None:
            raise RuntimeError('Unable to set autopilot params.')

        # Print any errors that occurred
        for response in future.result().results:
            if not response.successful:
                raise RuntimeError(f'Failed to set parameter: {response.reason}')

    def calculate_error(self):
        """
        Calculate the error between the state estimate and the commanded setpoint using the
        collected data.
        """

        # Get the estimate and command data from the right autopilot
        if self.current_autopilot == CurrentAutopilot.ROLL:
            estimate = np.array(self.state)[:, [0, 2]]
            command = np.array(self.internals_debug)[:, [0, 1]]
        elif self.current_autopilot == CurrentAutopilot.COURSE:
            estimate = np.array(self.state)[:, [0, 3]]
            command = np.array(self.commands)[:, [0, 2]]
        elif self.current_autopilot == CurrentAutopilot.PITCH:
            estimate = np.array(self.state)[:, [0, 4]]
            command = np.array(self.internals_debug)[:, [0, 2]]
        elif self.current_autopilot == CurrentAutopilot.ALTITUDE:
            estimate = np.array(self.state)[:, [0, 5]]
            command = np.array(self.commands)[:, [0, 3]]
        else:  # CurrentAutopilot.AIRSPEED
            estimate = np.array(self.state)[:, [0, 1]]
            command = np.array(self.commands)[:, [0, 1]]

        # Trim any measurements that happened before the first command
        estimate = estimate[np.where(estimate[:, 0] >= command[0, 0])[0], :]

        # Integrate across the square of the error
        cumulative_error = 0.0
        for i in range(estimate.shape[0] - 1):
            # Get command that was just previous
            command_index = np.where(command[:, 0] <= estimate[i + 1, 0])[0][-1]
            # Numerically integrate
            cumulative_error += (estimate[i + 1, 1] - command[command_index, 1])**2 \
                    * (estimate[i + 1, 0] - estimate[i, 0])

        return cumulative_error

    def get_next_gains(self):
        """
        Gets the next set of gains to test from either the queue or the optimizer.

        Returns:
        np.array, size num_gains, dtype float: The gains to test. [gain1, gain2, ...]
        """

        self.error_queue.put(self.calculate_error())
        self.state = []
        self.commands = []
        self.internals_debug = []

        if self.gain_queue.empty():
            # Empty the error queue and store it in a numpy array
            error_array = []
            while not self.error_queue.empty():
                error_array.append(self.error_queue.get())
            error_array = np.array(error_array)

            # Store the next set of gains in the queue
            for set in self.optimizer.get_next_parameter_set(error_array):
                self.gain_queue.put(set)

        return self.gain_queue.get()


def main(args=None):
    rclpy.init(args=args)

    autotune = Autotune()
    executor = MultiThreadedExecutor()
    executor.add_node(autotune)
    executor.spin()

    autotune.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
