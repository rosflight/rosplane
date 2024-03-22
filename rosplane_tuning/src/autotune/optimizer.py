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
        # Set initial values
        self.u1         = optimization_params[0]
        self.u2         = optimization_params[1]
        self.sigma      = optimization_params[2]
        self.init_alpha = optimization_params[3]
        self.tau        = optimization_params[4]
        self.flag       = 0 

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
