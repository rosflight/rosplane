#!/usr/bin/env python3

from enum import Enum, auto
import numpy as np


class OptimizerState(Enum):
    """
    This class defines the various states that exist within the optimizer.
    """
    UNINITIALIZED = auto()
    FINDING_GRADIENT = auto()
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
                when the inifity norm is less than tau.
        """

        # Save passed optimization parameters. See above for parameters details.
        self.mu_1  = optimization_params['mu_1']
        self.mu_2  = optimization_params['mu_2']
        self.sigma = optimization_params['sigma']
        self.alpha = optimization_params['alpha']
        self.tau   = optimization_params['tau']

        # The current state of the optimization process.
        self.state = OptimizerState.UNINITIALIZED

        # The jacobian of the function at x_0.
        self.J = None

        # The current search direction from x_0.
        self.p = None

        # The step size for calculating gradients with finite-difference.
        self.h = 0.01

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
        if self.state == OptimizerState.UNINITIALIZED:
            return "Optimizer awaiting first iteration."
        elif self.state == OptimizerState.FINDING_GRADIENT:
            return "Finding gradient."
        elif self.state == OptimizerState.BRACKETING:
            return "Bracketing step size."
        elif self.state == OptimizerState.PINPOINTING:
            return "Pinpointing step size."
        elif self.state == OptimizerState.TERMINATED:
            return "Optimizer terminated with final gains at: " + str(self.x_0)

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
            self.state = OptimizerState.FINDING_GRADIENT
            self.phi_0 = error[0]

            # Find the values for finite difference, where each dimmension is individually
            # stepped by h
            return np.tile(self.x_0, (self.x_0.shape[0], 1)) \
                    + np.identity(self.x_0.shape[0]) * self.h

        if self.state == OptimizerState.FINDING_GRADIENT:
            # Calculate the jacobian at the current point
            J_prior = self.J
            self.J = (error - self.phi_0) / self.h

            # Check for convergence using infinity norm
            if np.linalg.norm(self.J, np.inf) < self.tau:
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
            self.phi_2 = error[0]
            self.phi_2_prime = (error[1] - error[0]) / self.h
            new_points = self.bracketing()
            self.first_bracket = False
            return new_points

        elif self.state == OptimizerState.PINPOINTING:
            self.phi_2 = error[0]
            self.phi_2_prime = (error[1] - error[0]) / self.h
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
            self.state = OptimizerState.PINPOINTING
            return self.pinpointing()

        # Optimized
        if abs(self.phi_2_prime) <= -self.mu_2*self.phi_0_prime:
            self.x_0 = self.x_0 + self.alpha_2*self.p
            self.phi_0 = self.phi_2
            self.state = OptimizerState.FINDING_GRADIENT
            return np.tile(self.x_0, (self.x_0.shape[0], 1)) \
                    + np.identity(self.x_0.shape[0]) * self.h

        # Needs Pinpointing
        elif self.phi_2_prime >= 0:
            self.state = OptimizerState.PINPOINTING
            return self.pinpointing()

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

    def pinpointing(self):
        """
        This function conducts the pinpointing part of the optimization.

        Returns:
        alphastar (float): The optimal step size
        """

        alpha1 = self.distance(self.current_gains, gains1)
        alpha2 = self.distance(self.current_gains, gains2)
        alphap = self.distance(self.current_gains, gainsp)

        if alphap > phi1 + self.mu_1*alphap*phi1_prime or alphap > phi1:
            # Continue pinpointing
            gains2 = gainsp
            gainsp = self.interpolate(gains1, gains2)
            phi2 = phip
            phi2_prime = phip_prime
            self.save_gains = np.array([gains1, None, gains2, None, gainsp,
                                        [gain + self.h for gain in gainsp]])
            self.save_phis = np.array([phi1, phi1_prime, phi2, phi2_prime])
            new_gains = np.array([self.save_gains[4], self.save_gains[5]])
            return new_gains
        else:
            # Optimizedmu_2
            if abs(phip_prime) <= -self.u2*phi1_prime:
                self.state == OptimizerState.SELECT_DIRECTION
                alphastar = alphap
                new_gains = np.array([self.current_gains + alphastar*self.p])
                return new_gains
            # More parameterization needed
            elif phip_prime*(alpha2 - alpha1) >= 0:
                gains2 = gains1
                phi2 = phi1

            gains1 = gainsp
            phi1 = phip
            gainsp = self.interpolate(gains1, gains2)

            self.save_gains = np.array([gains1, None, gains2, None, gainsp,
                                        [gain + self.h for gain in gainsp]])
            self.save_phis = np.array([phi1, phi1_prime, phi2, phi2_prime])
            new_gains = np.array([self.save_gains[4], self.save_gains[5]])
            return new_gains

