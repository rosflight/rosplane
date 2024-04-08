#!/usr/bin/env python3

from enum import Enum, auto
import numpy as np


class OptimizerState(Enum):
    """
    This class defines the various states that exist within the optimizer.
    """
    SELECT_DIRECTION = auto()
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
        self.state = OptimizerState.FINDING_GRADIENT # Find the initial gradient from the starting gains
        self.initial_gains = initial_gains

        # Line Search Variables
        self.k                  = 0
        self.reset              = False
        self.p                  = np.array([])
        self.line_search_vector = np.array([0.0, 0.0])  # [prior_p, prior_phi_prime]
        self.save_gains         = np.array([])          # np.array to save gains between autotune iterations
        self.save_phis          = np.array([])          # np.array to save phis (to calc gradients) between autotune iterations, correspond with above array
        self.current_gains      = initial_gains         # Gains updated every succesful/nuclear pinpointing


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
        # Return with the final values, the best value found
        if self.state == OptimizerState.TERMINATED:
            return 'Optimization Terminated'
        else:
            return "Optimization in Progress"

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
            # Store current gains and error, request new gain and error for gradient, then select direction
            self.state = OptimizerState.SELECT_DIRECTION
            self.save_gains = self.current_gains
            self.save_phis = error
            new_gains = [gain + 0.01 for gain in self.initial_gains] 
            # After some calculations, found that even if you add 0.01 to all dimensions and divide by 0.01 for the gradient
            # you get roughly the same magnitude on the gradient. If we need more accuracy we can add a smarter value 
            # (decrease based on number of dimensions)
            return new_gains
        
        elif self.state == OptimizerState.SELECT_DIRECTION:
        
            if self.k == 0:
                new_gains = self.line_search(self.initial_gains, error, self.save_gains, self.save_phis)
            else:
                new_gains = self.line_search(self.current_gains, error, self.save_gains, self.save_phis)
            
            return new_gains
        
        elif self.state == OptimizerState.BRACKETING:
            # using error, conduct bracketing
            # return the next set of gains to test
            gains = self.save_gains[0]
            gainsh = self.save_gains[1]
            gains2 = self.save_gains[2]
            gains2h = self.save_gains[3]

            new_gains = self.bracketing(gains, gainsh, self.save_phis[0], self.save_phis[1], gains2, gains2h, error[0], error[1])
            return new_gains
        
        elif self.state == OptimizerState.PINPOINTING:

            gains1  = self.save_gains[0]
            gains2  = self.save_gains[2]
            gainsp  = self.save_gains[4]

            phip = error[0]
            phiph = error[1]
            phip_prime = self.get_gradient(phip, phiph)

            phi1 = self.save_phis[0]
            phi1_prime = self.save_phis[1]

            new_gains = self.pinpointing(gains1, phi1, phi1_prime, gains2, gainsp, phip, phip_prime)
            return new_gains

        else:  # Process Terminated
            return self.current_gains

    def get_gradient(self, phi, phih):
        """
        This function returns the gradient at the given point using forward finite difference.

        Parameters:
        phi (float): The function evaluation at the point of the gradient.
        phih (float): The function evaluation at a point slightly offset from the point. 

        Returns:
        float: The gradient at the given point.
        """
        return (phih - phi) / 0.01  # Shouldn't be a hard coded value, replace with a variable

    def line_search(self, gains, phi, gainsh, phih):
        """
        This function conducts the initial direction selection part of the optimization,
        using conjugate gradient descent.

        Parameters:
        gains (np.array, size num_gains, dtype float): The current gains. [gain1, gain2, ...]
        phi (np.array, size num_gains, dtype float): The error from the list of gains. [error1, error2, ...]
        phi_prime (): The gradient of the error from the list of gains. 

        Returns:
        np.array, size num_gains*num_sets, dtype float: The next series of gains to be tested. Any
            number of gain sets can be returned, depending on the number of gains that need to be tested for 
            the follow on optimization. [[gain1_1, gain2_1, ...], [gain1_2, gain2_2, ...], ...] 
        """
        
        phi_prime = self.get_gradient(phi, phih) # Calculate gradient

        # Check for convergence
        if np.linalg.norm(phi_prime,np.inf) < self.tau:
            self.state = OptimizerState.TERMINATED
            return self.current_gains

        # Select Direction
        prior_p = self.line_search_vector[0]
        prior_phi_prime = self.line_search_vector[1]

        if self.k == 0 or self.reset == True:
            self.reset = False
            self.p = -phi_prime/np.linalg.norm(phi_prime)
        else:
            bk = np.dot(phi_prime,phi_prime)/np.dot(prior_phi_prime,prior_phi_prime)
            self.p = -phi_prime/np.linalg.norm(phi_prime) + bk*prior_p
        
        # Prepare for bracketing
        self.state = OptimizerState.BRACKETING
        # Request phi2 and phi2+h
        gains2 = gains + self.init_alpha*self.p
        gains2h = [gain + 0.01 for gain in gains2]
        new_gains = np.array([gains2, gains2h])
        
        # Save phi1 and phi1+h
        self.save_gains = np.array([gains, gainsh, gains2, gains2h])
        self.save_phis = np.array([phi, phih])
        self.k += 1

        return new_gains

    def distance(point1, point2):
        """
        This function calculates the distance between two points.

        Parameters:
        point1 (np.array, size num_gains, dtype float): The first point. [gain1, gain2, ...]
        point2 (np.array, size num_gains, dtype float): The second point. [gain1, gain2, ...]

        Returns:
        float: The distance between the two points.
        """
        return np.linalg.norm(point1 - point2)

    def bracketing(self, gains, gainsh, phi1, phi1_prime, gains2, gains2h, phi2, phi2_prime):
        """
        This function conducts the bracketing part of the optimization.
        """

        alpha1 = self.distance(self.init_gains, gains)  # For the initial pass, gains should be initial_gains, so alpha1 = 0
        alpha2 = self.distance(self.init_gains, gains2)

        # Needs Pinpointing
        if(phi2 > phi1 + self.u1*self.init_alpha*phi1_prime) or (phi2 > phi1):
            self.state == OptimizerState.PINPOINTING
            # Save old points
            self.save_gains = np.array([gains, gainsh, gains2, gains2h, gainsp, [gain + 0.01 for gain in gainsp]])
            self.save_phis = np.array([phi1, phi1_prime, phi2, phi2_prime])
            # Request new point
            alphap = self.interpolate(alpha1, alpha2)
            gainsp = self.init_gains + alphap*self.p
            new_gains = np.array([self.save_gains[4], self.save_gains[5]])
            return new_gains

        # Optimized
        if abs(phi2_prime) <= -self.u2*phi1_prime:
            self.state == OptimizerState.SELECT_DIRECTION
            new_gains = np.array([self.init_gains + alpha2*self.p])
            self.current_gains = new_gains
            return new_gains

        # Needs Pinpointing
        elif phi2_prime >= 0:
            self.state == OptimizerState.PINPOINTING
            # Save old points
            self.save_gains = np.array([gains, gainsh, gains2, gains2h, gainsp, [gain + 0.01 for gain in gainsp]])
            self.save_phis = np.array([phi1, phi1_prime, phi2, phi2_prime])
            # Request new point
            alphap = self.interpolate(alpha1, alpha2)
            gainsp = self.init_gains + alphap*self.p
            new_gains = np.array([self.save_gains[4], self.save_gains[5]])
            return new_gains

        # Needs more Bracketing
        else:
            alpha1 = alpha2
            alpha2 = self.sigma*alpha2

            # Request new points
            gains1 = self.init_gains + alpha1*self.p
            gains2 = self.init_gains + alpha2*self.p
            gains1h = [gain + 0.01 for gain in gains1]
            gains2h = [gain + 0.01 for gain in gains2]

            new_gains = np.array([gains1, gains1h, gains2, gains2h])
            return new_gains

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

    def pinpointing(self, gains1, phi1, phi1_prime, gains2, gainsp, phip, phip_prime):
        """
        This function conducts the pinpointing part of the optimization.

        Parameters:

        Returns:
        alphastar (float): The optimal step size
        """

        alpha1 = self.distance(self.init_gains, gains1)
        alpha2 = self.distance(self.init_gains, gains2)
        alphap = self.distance(self.init_gains, gainsp)

        if alphap > phi1 + self.u1*alphap*phi1_prime or alphap > phi1:
            # Continue pinpointing
            gains2 = gainsp
            gainsp = self.interpolate(gains1, gains2)
            phi2 = phip
            phi2_prime = phip_prime
            self.save_gains = np.array([gains1, None, gains2, None, gainsp, [gain + 0.01 for gain in gainsp]])
            self.save_phis = np.array([phi1, phi1_prime, phi2, phi2_prime])
            new_gains = np.array([self.save_gains[4], self.save_gains[5]])
            return new_gains
        else:
            # Optimized
            if abs(phip_prime) <= -self.u2*phi1_prime:
                self.state == OptimizerState.SELECT_DIRECTION
                alphastar = alphap
                new_gains = np.array([self.init_gains + alphastar*self.p])
                return new_gains
            # More parameterization needed
            elif phip_prime*(alpha2 - alpha1) >= 0:
                gains2 = gains1
                phi2 = phi1
                
            gains1 = gainsp
            phi1 = phip
            gainsp = self.interpolate(gains1, gains2)

            self.save_gains = np.array([gains1, None, gains2, None, gainsp, [gain + 0.01 for gain in gainsp]])
            self.save_phis = np.array([phi1, phi1_prime, phi2, phi2_prime])
            new_gains = np.array([self.save_gains[4], self.save_gains[5]])
            return new_gains

        # Check for failure criteria - the nuclear option
            # self.state == OptimizerState.SELECT_DIRECTION
            # self.current_gains = new_gains
