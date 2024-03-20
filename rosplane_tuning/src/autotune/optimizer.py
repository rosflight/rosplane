#!/usr/bin/env python3


class Optimizer:
    """
    This class contains the optimization algorithms used by the autotune node. It is able to
    optimize functions of any number of parameters.
    """

    def __init__(self, initial_gains, optimization_params): 
        """
        Parameters:
        initial_gains (list): The starting point for the gains to be optimized (x0).
        optimization_params (list): Parameters needed for the optimization operation.   
        """
        self.u1         = optimization_params[0]
        self.u2         = optimization_params[1]
        self.sigma      = optimization_params[2]
        self.init_alpha = optimization_params[3]
        self.tau        = optimization_params[4]
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

    def calculate_error(self):
        """
        Calculate the error between the state estimate and the commanded setpoint using the
        collected data.
        """
        # TODO: Implement this function
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

    def bracketing(self, phi1, phi1_prime, phi2, phi2_prime):
        """
        This function conducts the bracketing part of the optimization.

        Parameters:
        phi1 (float): The value of the function at point 1.
        phi1_prime (float): The gradient of the function at point 1.
        phi2 (float): The value of the function at point 2.
        phi2_prime (float): The gradient of the function at point 2.

        Returns:
        alphastar (float): The optimal step size
        """

        alpha2 = self.init_alpha
        first = True
        if(phi2 > phi1 + self.u1*self.init_alpha*phi1_prime) or (first == False and phi2 > phi1):
            # alphastar = pinpointing()
            alphastar = 1 # Testing
            return alphastar
        
        if abs(phi2_prime) <= -self.u2*phi1_prime:
            alphastar = alpha2
            return alphastar
        
        elif phi2_prime >= 0:
            # alphastar = pinpointing()
            alphastar = 1 # Testing
            return alphastar
        
        else:
            alpha1 = alpha2
            alpha2 = self.sigma*alpha2

        first = False

    def pinpointing(self, alphalow, alphahigh, philow, philow_prime, phihigh, phihigh_prime):
        """
        This function conducts the pinpointing part of the optimization.

        Parameters:
        alpha1 (float): The first distance.
        alpha2 (float): The second distance.
        phi1 (float): The value of the function at point 1.
        phi1_prime (float): The gradient of the function at point 1.
        phi2 (float): The value of the function at point 2.
        phi2_prime (float): The gradient of the function at point 2.
        
        Returns:
        alphastar (float): The optimal step size
        """
        alphap = self.interpolate(alphalow, alphahigh)
        # Calculate phip
        phip = 1
        # Calculate phip_prime
        phip_prime = 1

        if alphap > philow + self.u1*alphap*philow_prime or alphap > philow:
            alphahigh = alphap
            phihigh = phip
        else:
            if abs(phip_prime) <= -self.u2*philow_prime:
                alphastar = alphap
                return alphastar
            
            elif phip_prime*(alphahigh - alphalow) >= 0:
                alphahigh = alphalow
            
            alphalow = alphap