#!/usr/bin/env python3

from enum import Enum, auto
import numpy as np


class OptimizerState(Enum):
    """
    This class defines the various states that exist within the optimizer.
    """
    FINDING_GRADIENT = auto()
    BRACKETING = auto()
    PINPOINTING = auto()
    TERMINATED = auto()


class Optimizer:
    """
    This class contains the optimization algorithms used by the autotune node. It is able to
    optimize functions of any number of parameters.
    """

    def __init__(self, initial_gains, optimization_params): 
        """
        Parameters:
        initial_gains (np.array, size num_gains, dtype float): The starting point for the gains to
            be optimized (x0). [gain1, gain2, ...]
        optimization_params (dictionary): Parameters needed for the optimization operation.
            - u1 (float): 1st Strong Wolfe Condition, must be between 0 and 1.
            - u2 (float): 2nd Strong Wolfe Condition, must be between u1 and 1.
            - sigma (float): The step size multiplier.
            - alpha (float): The initial step size, usually 1.
            - tau (float): Convergence tolerance, usually 1e-3.
        """
        # Set initial values
        self.u1         = optimization_params['u1']
        self.u2         = optimization_params['u2']
        self.sigma      = optimization_params['sigma']
        self.init_alpha = optimization_params['alpha']
        self.tau        = optimization_params['tau']
        self.flag       = 0
        self.state = OptimizerState.FINDING_GRADIENT

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

    def get_optimiztion_status(self):
        """
        This function returns the status of the optimization algorithm.

        Returns:
        str: The status of the optimization algorithm.
        """
        return 'TODO: Status not implemented.'

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

        if self.state == OptimizerState.FINDING_GRADIENT:
            pass
        elif self.state == OptimizerState.BRACKETING:
            pass
        elif self.state == OptimizerState.PINPOINTING:
            pass
        else:  # Terminated
            pass


    def get_gradient(self, fx, fxh):
        """
        This function returns the gradient at the given point using forward finite difference.

        Parameters:
        fx (float): The function evaluation at the point of the gradient.
        fxh (float): The function evaluation at a point slightly offset from the point. 

        Returns:
        float: The gradient at the given point.
        """
        return (fxh - fx) / 0.01

    def next_step(self, phi1, phi1_prime, phi2, phi2_prime):
        """
        This function chooses which function to run next based on the flag.
        """
        if self.flag == 0:
            self.initial_bracketing(phi1, phi1_prime, phi2, phi2_prime)
        pass

    # Flag 0 - Initial Bracketing
    def initial_bracketing(self, phi1, phi1_prime, phi2, phi2_prime):
        """
        This function conducts the bracketing part of the optimization.
        """

        alpha1 = 0
        alpha2 = self.init_alpha

        # Needs Pinpointing
        if(phi2 > phi1 + self.u1*self.init_alpha*phi1_prime):
            flag = 2
            alphap = self.interpolate(alpha1, alpha2)
            return alpha2

        # Optimized
        if abs(phi2_prime) <= -self.u2*phi1_prime:
            flag = 4
            return alpha2

        # Needs Pinpointing
        elif phi2_prime >= 0:
            flag = 2
            alphap = self.interpolate(alpha1, alpha2)
            return alpha2

        # Needs more Bracketing
        else:
            alpha1 = alpha2
            alpha2 = self.sigma*alpha2
            flag = 1
            return alpha2


    # Flag 1 - Bracketing
    def bracketing(self, phi1, phi1_prime, phi2, phi2_prime, temp_alpha1, temp_alpha2):
        """
        This function conducts the bracketing part of the optimization.
        """
        alpha1 = temp_alpha1
        alpha2 = temp_alpha2

        # Needs Pinpointing
        if(phi2 > phi1 + self.u1*self.init_alpha*phi1_prime) or (phi2 > phi1):
            flag = 2
            alphap = self.interpolate(alpha1, alpha2)
            res = [alpha1, alphap, alpha2]
            return res 

        # Optimized
        if abs(phi2_prime) <= -self.u2*phi1_prime:
            flag = 4
            res = [alpha2]
            return res

        # Needs Pinpointing
        elif phi2_prime >= 0:
            flag = 2
            alphap = self.interpolate(alpha1, alpha2)
            res = [alpha1, alphap, alpha2]
            return res

        # Needs more Bracketing
        else:
            alpha1 = alpha2
            alpha2 = self.sigma*alpha2
            res = [alpha1, alpha2]
            return res

    # Interpolating
    def interpolate(self, alpha1, alpha2):
        """
        This function interpolates between two points.

        Parameters:
        alpha1 (float): The first distance.
        alpha2 (float): The second distance.

        Returns:
        float: The interpolated value or alphap.
        """
        return (alpha1 + alpha2)/2

    # Flag 2 - Pinpointing 
    def pinpointing(self, alpha1, phi1, phi1_prime, alpha2, phi2, phi2_prime, alphap, phip, phip_prime):
        """
        This function conducts the pinpointing part of the optimization.

        Parameters:
        alpha1 (float): The first distance.
        phi1 (float): The value of the function at point 1.
        phi1_prime (float): The gradient of the function at point 1.
        alpha2 (float): The second distance.
        phi2 (float): The value of the function at point 2.
        phi2_prime (float): The gradient of the function at point 2.

        Returns:
        alphastar (float): The optimal step size
        """

        if alphap > phi1 + self.u1*alphap*phi1_prime or alphap > phi1:
            alpha2 = alphap
            phi2 = phip
            res = [alpha1, alpha2]
            return res
        else:
            # Optimized
            if abs(phip_prime) <= -self.u2*phi1_prime:
                flag = 4
                alphastar = alphap
                res = [alphastar]
                return res
            # More parameterization needed
            elif phip_prime*(alpha2 - alpha1) >= 0:
                alpha2 = alpha1

            alpha1 = alphap
            res = [alpha1, alpha2]
            return res

        # Check for failure criteria

    # Flag 3 - Failure Convergence
    def failure(self):
        pass

    # Flag 4 - Successful Convergence
    def success(self):
        pass
