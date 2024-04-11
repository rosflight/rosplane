#!/usr/bin/env python3

from enum import Enum, auto
import numpy as np


class OptimizerState(Enum):
    """
    This class defines the various states that exist within the optimizer.
    """
    UNINITIALIZED = auto()
    FINDING_JACOBEAN = auto()
    BRACKETING = auto()
    PINPOINTING = auto()
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
            - mu_1 (float): 1st Strong Wolfe Condition, specifies the minimum decrease that will
                be accepted. Must be between 0 and 1, but is usually a very small value like 1e-4.
            - mu_2 (float): 2nd Strong Wolfe Condition, specifies the sufficient decrease in
                curvature that will be accepted. Must be between mu_1 and 1, typically between 0.1
                and 0.9.
            - sigma (float): The step size multiplier, must be greater than 1.
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
        self.mu_1  = optimization_params['mu_1']
        self.mu_2  = optimization_params['mu_2']
        self.sigma = optimization_params['sigma']
        self.alpha = optimization_params['alpha']
        self.tau   = optimization_params['tau']
        self.h     = optimization_params['h']

        # The current state of the optimization process.
        self.state = OptimizerState.UNINITIALIZED

        # The jacobean of the function at x_0.
        self.J = None

        # The current search direction from x_0.
        self.p = None

        # The current best point in the optimization, where we are stepping away from.
        self.x_0 = initial_point
        # The function value (error) at x_0.
        self.phi_0 = None
        # The gradient at x_0 along the search direction p.
        self.phi_0_prime = None

        # The function value at the best point on the line search (x_1).
        self.phi_1 = None
        # The gradient at x_1 along the search direction p.
        self.phi_1_prime = None
        # The step size from x_0 to x_1 along the search direction p.
        self.alpha_1 = None

        # The function value at the best point on the line search (x_2).
        self.phi_2 = None
        # The gradient at x_2 along the search direction p.
        self.phi_2_prime = None
        # The step size from x_0 to x_2 along the search direction p.
        self.alpha_2 = None

        # Bool to specify if this is the first bracketing step, used by the bracketing algorithm.
        self.first_bracket = None

        # TODO: describe these parameters
        self.alpha_p = None
        self.phi_p = None
        self.phi_p_prime = None
        self.alpha_low = None
        self.alpha_high = None
        self.phi_low = None
        self.phi_low_prime = None
        self.phi_high = None
        self.phi_high_prime = None

        # Number of iterations since improvement
        self.k = 0
        self.max_k = 5

        # Reason for termination
        self.reason = ""

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
        elif self.state == OptimizerState.FINDING_JACOBEAN:
            return "Finding gradient."
        elif self.state == OptimizerState.BRACKETING:
            return "Bracketing step size."
        elif self.state == OptimizerState.PINPOINTING:
            return "Pinpointing step size."
        elif self.state == OptimizerState.TERMINATED:
            return str(self.reason) + " Final gains: " + str(self.x_0)

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
            self.reason = f"{self.k} iterations without improvement."
            self.state = OptimizerState.TERMINATED
            return self.x_0.reshape(1, 2)

        if self.state == OptimizerState.UNINITIALIZED:
            # Find the gradient at the new point
            self.state = OptimizerState.FINDING_JACOBEAN
            self.phi_0 = error[0]

            # Find the values for finite difference, where each dimension is individually
            # stepped by h
            return np.tile(self.x_0, (self.x_0.shape[0], 1)) \
                    + np.identity(self.x_0.shape[0]) * self.h

        if self.state == OptimizerState.FINDING_JACOBEAN:
            self.k = 0

            # Calculate the jacobean at the current point
            J_prior = self.J
            self.J = (error - self.phi_0) / self.h

            # Check for convergence using infinity norm
            if np.linalg.norm(self.J, np.inf) < self.tau:
                self.reason = "Converged."
                self.state = OptimizerState.TERMINATED
                return self.x_0.reshape(1, 2)

            # Find the search direction
            if J_prior is None:
                self.p = -self.J / np.linalg.norm(self.J)
            else:
                beta = np.dot(self.J, self.J) / np.dot(J_prior, J_prior)
                self.p = -self.J / np.linalg.norm(self.J) + beta*self.p

            # Begin bracketing
            self.phi_0_prime = np.dot(self.p, self.J)
            self.alpha_1 = 0
            self.alpha_2 = self.alpha
            self.phi_1 = self.phi_0
            self.phi_1_prime = self.phi_0_prime
            self.first_bracket = True
            self.state = OptimizerState.BRACKETING

            # Find phi_2 and phi_2_prime
            x_2 = (self.x_0 + self.alpha_2*self.p)
            return np.array([x_2, (x_2 + self.h*self.p)])

        elif self.state == OptimizerState.BRACKETING:
            self.k += 1
            self.phi_2 = error[0]
            self.phi_2_prime = (error[1] - error[0]) / self.h
            new_points = self.bracketing()
            self.first_bracket = False
            return new_points

        elif self.state == OptimizerState.PINPOINTING:
            self.k += 1
            self.phi_p = error[0]
            self.phi_p_prime = (error[1] - error[0]) / self.h
            return self.pinpointing()

        else:  # Process Terminated
            return self.x_0.reshape(1, 2)


    def bracketing(self):
        """
        This function conducts the bracketing part of the optimization.
        """

        # Needs Pinpointing
        if(self.phi_2 > self.phi_0 + self.mu_1*self.alpha_2*self.phi_0_prime) \
                or (not self.first_bracket and self.phi_2 > self.phi_1):
            self.alpha_low = self.alpha_1
            self.alpha_high = self.alpha_2
            self.phi_low = self.phi_1
            self.phi_low_prime = self.phi_1_prime
            self.phi_high = self.phi_2
            self.phi_high_prime = self.phi_2_prime
            self.state = OptimizerState.PINPOINTING

            self.alpha_p = interpolate(self.alpha_1, self.alpha_2, self.phi_1, self.phi_2,
                                       self.phi_1_prime, self.phi_2_prime)
            x_p = self.x_0 + self.alpha_p*self.p
            return np.array([x_p, (x_p + self.h*self.p)])

        # Optimized
        if abs(self.phi_2_prime) <= -self.mu_2*self.phi_0_prime:
            self.x_0 = self.x_0 + self.alpha_2*self.p
            self.phi_0 = self.phi_2
            self.state = OptimizerState.FINDING_JACOBEAN
            self.alpha = self.alpha_2
            return np.tile(self.x_0, (self.x_0.shape[0], 1)) \
                    + np.identity(self.x_0.shape[0]) * self.h

        # Needs Pinpointing
        elif self.phi_2_prime >= 0:
            self.alpha_low = self.alpha_2
            self.alpha_high = self.alpha_1
            self.phi_low = self.phi_2
            self.phi_low_prime = self.phi_2_prime
            self.phi_high = self.phi_1
            self.phi_high_prime = self.phi_1_prime
            self.state = OptimizerState.PINPOINTING

            self.alpha_p = interpolate(self.alpha_1, self.alpha_2, self.phi_1, self.phi_2,
                                       self.phi_1_prime, self.phi_2_prime)
            x_p = self.x_0 + self.alpha_p*self.p
            return np.array([x_p, (x_p + self.h*self.p)])

        # Needs more Bracketing
        else:
            # Set x_2 to x_1 and increase alpha_2
            self.alpha_1 = self.alpha_2
            self.alpha_2 = self.sigma*self.alpha_2
            self.phi_1 = self.phi_2
            self.phi_1_prime = self.phi_2_prime

            # Find new x_2
            x_2 = self.x_0 + self.alpha_2*self.p
            return np.array([x_2, (x_2 + self.h*self.p)])

    def pinpointing(self):
        """
        This function conducts the pinpointing part of the optimization.
        """

        if (self.phi_p > self.phi_0 + self.mu_1*self.alpha_p*self.phi_0_prime) \
                or (self.phi_p > self.phi_low):
            # Set interpolated point to high, continue
            self.alpha_high = self.alpha_p
            self.phi_high = self.phi_p
            self.phi_high_prime = self.phi_p_prime
        else:
            if abs(self.phi_p_prime) <= -self.mu_2*self.phi_0_prime:
                # Pinpointing complete, set interpolated point to x_0 and find new jacobian
                self.x_0 = self.x_0 + self.alpha_p*self.p
                self.phi_0 = self.phi_p
                self.state = OptimizerState.FINDING_JACOBEAN
                self.alpha = self.alpha_p
                return np.tile(self.x_0, (self.x_0.shape[0], 1)) \
                    + np.identity(self.x_0.shape[0]) * self.h

            elif self.phi_p_prime*(self.alpha_high - self.alpha_low) >= 0:
                # Set high to low (and low to interpolated)
                self.alpha_high = self.alpha_low
                self.phi_high = self.phi_low
                self.phi_high_prime = self.phi_low_prime

            # Set low to interpolated
            self.alpha_low = self.alpha_p
            self.phi_low = self.phi_p
            self.phi_low_prime = self.phi_p_prime

        # Find value and gradient at interpolated point
        self.alpha_p = interpolate(self.alpha_low, self.alpha_high, self.phi_low, self.phi_high,
                                   self.phi_low_prime, self.phi_high_prime)
        x_p = self.x_0 + self.alpha_p*self.p
        return np.array([x_p, (x_p + self.h*self.p)])


def interpolate(alpha_1, alpha_2, phi_1, phi_2, phi_1_prime, phi_2_prime):
    """
    This function performs quadratic interpolation between two points to find the minimum,
    given their function values and derivatives.

    Parameters:
    alpha_1 (float): Function input 1.
    alpha_2 (float): Function input 2.
    phi_1 (float): Function value at alpha_1.
    phi_2 (float): Function value at alpha_2.
    phi_1_prime (float): Function derivative at alpha_1.
    phi_2_prime (float): Function derivative at alpha_2.

    Returns:
    float: The function input (alpha_p) at the interpolated minimum.
    """

    beta_1 = phi_1_prime + phi_2_prime - 3*(phi_1 - phi_2) / (alpha_1 - alpha_2)
    beta_2 = np.sign(alpha_2 - alpha_1) * np.sqrt(beta_1**2 - phi_1_prime*phi_2_prime)
    return alpha_2 - (alpha_2 - alpha_1) \
            * (phi_2_prime + beta_2 - beta_1) / (phi_2_prime - phi_1_prime + 2*beta_2)
