from optimizer import Optimizer
import numpy as np


# Function to test the optimizer with
def function(x):
    # Matyas function
    return 0.26 * (x[0] ** 2 + x[1] ** 2) - 0.48 * x[0] * x[1]


# Initialize optimizer
points = np.array([[1, 2]])  # Initial point
optimization_params = {'u1': 1e-4,
                       'u2': 0.5,
                       'sigma': 1.5,
                       'alpha': 1.0,
                       'tau': 1e-3}
optimizer = Optimizer(points[0], optimization_params)


while not optimizer.optimization_terminated():
    # Print status
    print(optimizer.get_optimiztion_status())

    # Calculate error for current points
    error = []
    for point in points:
        error.append(function(point))
    error = np.array(error)

    # Pass points to optimizer
    points = optimizer.get_next_parameter_set(error)


print('Optimization terminated with status: {}'.format(optimizer.get_optimiztion_status()))

