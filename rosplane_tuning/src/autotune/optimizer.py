#!/usr/bin/env python3

from enum import Enum, auto
import numpy as np


class OptimizerState(Enum):
    """
    This class defines the various states that exist within the optimizer.
    """
    UNINITIALIZED = auto()
    FIRST_ITERATION = auto()
    OPTIMIZING = auto()
    TERMINATED = auto()


class Optimizer:
    """
    This class contains the optimization algorithms used by the autotune node. It is able to
    optimize functions of any number of parameters.
    """

    def __init__(self, initial_point, optimization_params):
        """
        Parameters:
        initial_point (np.array, size num_gains, dtype float): The starting point for the gains to
            be optimized (x_0). [gain1, gain2, ...]
        optimization_params (dictionary): Parameters needed for the optimization operation.
            - alpha (float): The step size.
            - h (float): Finite difference step size, used to calculate the gradient.
        """

        # Save passed optimization parameters. See above for parameters details.
        self.alpha = optimization_params['alpha']
        self.h     = optimization_params['h']

        # The current state of the optimization process.
        self.state = OptimizerState.UNINITIALIZED

        # Reason for termination
        self.reason = ""

        # Initialize the gains to the initial point
        self.x = initial_point
        self.prev_error = None


    def optimization_terminated(self):
        """
        This function checks if the optimization algorithm has terminated.

        Returns:
        bool: True if optimization is terminated, False otherwise.
        """
        if self.state == OptimizerState.TERMINATED:
            return True
        else:
            return False

    def get_optimization_status(self):
        """
        This function returns the status of the optimization algorithm.

        Returns:
        str: The status of the optimization algorithm.
        """
        if self.state == OptimizerState.UNINITIALIZED:
            return "Optimizer awaiting first iteration."
        elif self.state == OptimizerState.FIRST_ITERATION:
            return "First iteration."
        elif self.state == OptimizerState.OPTIMIZING:
            return "Optimizing."
        else:  # self.state == OptimizerState.TERMINATED:
            return str(self.reason) + " Final gains: " + str(self.x)

    def get_next_parameter_set(self, error):
        """
        This function returns the next set of gains to be tested by the optimization algorithm.
        Needs to return valid array even if optimization is terminated.

        Parameters:
        error (np.array, size num_gains, dtype float): The error from the list of gains to test that
            was returned by this function previously. If this is the first iteration, the error of
            the initial gains will be given. [error1, error2, ...]

        Returns:
        np.array, size num_gains*num_sets, dtype float: The next series of gains to be tested. Any
            number of gain sets can be returned, but each set should correspond to the initial gains
            given at initialization. [[gain1_1, gain2_1, ...], [gain1_2, gain2_2, ...], ...]
        """

        if self.state == OptimizerState.UNINITIALIZED:
            # Find the gradient at the new point
            self.state = OptimizerState.FIRST_ITERATION
            self.prev_error = error.item(0)

            # Find the values for finite difference, where each dimension is individually
            # stepped by h
            return np.tile(self.x, (self.x.shape[0], 1)) \
                    + np.identity(self.x.shape[0]) * self.h

        if self.state == OptimizerState.FIRST_ITERATION or self.state == OptimizerState.OPTIMIZING:
            # Calculate the jacobean at the current point
            if self.state == OptimizerState.FIRST_ITERATION:
                J = (error - self.prev_error) / self.h
                self.state = OptimizerState.OPTIMIZING
            else:
                J = (error[1:] - error[0]) / self.h

                # Check if any of the new points are better
                if np.min(error) > self.prev_error:
                    self.reason = "Current iteration worst than previous."
                    self.state = OptimizerState.TERMINATED
                    return self.x.reshape(1, 2)
                self.prev_error = error.item(0)


            # Take a step in the search direction
            self.x = self.x - J * self.alpha

            # Find the gradient at the new point
            points = np.tile(self.x, (self.x.shape[0], 1)) \
                    + np.identity(self.x.shape[0]) * self.h
            points = np.vstack((self.x, points))
            return points

        else:  # Process Terminated
            return self.x.reshape(1, 2)

