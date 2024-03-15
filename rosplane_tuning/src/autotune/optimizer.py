#!/usr/bin/env python3


class Optimizer:
    """
    This class contains the optimization algorithms used by the autotune node. It is able to
    optimize functions of any number of parameters.
    """

    def __init__(self, initial_gains):  # TODO: Add any other optimization parameters needed (e.g. step size, termination criteria)
        """
        Parameters:
        initial_gains (list): The starting point for the gains to be optimized (x0).
        """
        pass

    def optimization_terminated(self):
        """
        This function checks if the optimization algorithm has terminated.

        Returns:
        bool: True if optimization should terminate, False otherwise
        """
        pass

    def get_optimiztion_status(self):
        """
        This function returns the status of the optimization algorithm.

        Returns:
        str: The status of the optimization algorithm
        """
        pass

    def get_next_parameter_set(self, error):
        """
        This function returns the next set of gains to be tested by the optimization algorithm.

        Parameters:
        error (float): The error from the set of gains returned previously. If this is the first
        iteration, the error of the initial gains will be given.

        Returns:
        list: The next set of gains to be tested. Should be the same length as the initial gains.
        """
        pass

