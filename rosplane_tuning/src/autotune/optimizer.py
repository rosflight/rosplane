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
            - alpha (float): The initial step size. The ideal value for this is very
                application-specific.
            - tau (float): The convergence tolerance. The optimization is considered converged
                when the infinity norm is less than tau.
            - h (float): Finite difference step size. This is used to calculate the gradient
                by stepping slightly away from a point and then calculating the slope of the
                line between the two points. Ideally this value will be small, but it is sensitive
                to noise and can't get too small.
        """

        # Save passed optimization parameters. See above for parameters details.
        self.alpha = optimization_params['alpha']
        self.tau   = optimization_params['tau']
        self.h     = optimization_params['h']

        # The current state of the optimization process.
        self.state = OptimizerState.UNINITIALIZED

        # Reason for termination
        self.reason = ""

        # Initialize the gains to the initial point
        self.x = initial_point
        self.f_initial = None

        # Keep track of the number of iterations
        self.k = 0
        self.max_k = 20

        # Initialize the velocity
        self.v = np.zeros([self.x.shape[0]]).T

        self.inertia = 0.8
        self.drag = 0.9

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
        elif self.state == OptimizerState.OPTIMIZING:
            return "Optimizing."
        elif self.state == OptimizerState.TERMINATED:
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

        if self.k >= self.max_k:
            self.reason = "Interations complete."
            self.state = OptimizerState.TERMINATED
            return self.x.reshape(1, 2)

        if self.state == OptimizerState.UNINITIALIZED:
            # Find the gradient at the new point
            self.state = OptimizerState.FIRST_ITERATION
            self.f_initial = error.item(0)

            # Find the values for finite difference, where each dimension is individually
            # stepped by h
            return np.tile(self.x, (self.x.shape[0], 1)) \
                    + np.identity(self.x.shape[0]) * self.h

        if self.state == OptimizerState.FIRST_ITERATION or self.state == OptimizerState.OPTIMIZING:
            self.k += 1
            # Calculate the jacobean at the current point
            if self.state == OptimizerState.FIRST_ITERATION:
                J = (error - self.f_initial) / self.h
                self.state = OptimizerState.OPTIMIZING
            else:
                J = (error[1:] - error[0]) / self.h
            J_norm = J / np.linalg.norm(J)

            # Update the velocity
            self.v = self.inertia * self.v + (1 - self.inertia) * -J_norm

            # Slow the velocity down with drag
            self.v = self.v * self.drag

            # Take a step in the search direction
            self.x = self.x + self.v * self.alpha

            # Find the gradient at the new point
            points = np.tile(self.x, (self.x.shape[0], 1)) \
                    + np.identity(self.x.shape[0]) * self.h
            points = np.vstack((self.x, points))
            return points

        else:  # Process Terminated
            return self.x.reshape(1, 2)

