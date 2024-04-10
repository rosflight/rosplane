from optimizer import Optimizer

import numpy as np
import matplotlib.pyplot as plt


# Function to test the optimizer with
def Matyas(x):
    # Matyas function
    return 0.26 * (x[0] ** 2 + x[1] ** 2) - 0.48 * x[0] * x[1]

def Circle(x):
    # Circle function
    return x[0] ** 2 + x[1] ** 2

function = Circle
function = Matyas

# Initialize optimizer
curr_points = np.array([[0.0, 2.0]])  # Initial point
optimization_params = {'mu_1': 1e-4,
                       'mu_2': 0.5,
                       'sigma': 1.5,
                       'alpha': 1.0,
                       'tau': 1e-2,
                       'h': 1e-2}
optimizer = Optimizer(curr_points[0], optimization_params)


# Run optimization
all_points = []
x_0_points = []
while not optimizer.optimization_terminated():
    # Print status
    print(optimizer.get_optimization_status())

    # Calculate error for current points
    error = []
    for point in curr_points:
        error.append(function(point))
    error = np.array(error)
    # Pass points to optimizer
    curr_points = optimizer.get_next_parameter_set(error)

    # Store points 
    for point in curr_points:
        all_points.append(point)
    x_0_points.append(optimizer.x_0)

    # End interation step
all_points = np.array(all_points)
x_0_points = np.array(x_0_points)

print('Optimization terminated with status: {}'.format(optimizer.get_optimization_status()))


# Plot the function with the optimization path
x_min = np.min(all_points[:, 0]) - 1
x_max = np.max(all_points[:, 0]) + 1
y_min = np.min(all_points[:, 1]) - 1
y_max = np.max(all_points[:, 1]) + 1
x = np.linspace(x_min, x_max, 100)
y = np.linspace(y_min, y_max, 100)
X, Y = np.meshgrid(x, y)
Z = function([X, Y])
plt.contour(X, Y, Z, 50)
plt.scatter(x_0_points[:, 0], x_0_points[:, 1], color='g', marker='x', s=200)
plt.plot(all_points[:, 0], all_points[:, 1], marker='o', color='r')
# Lock the x and y axis to be equal
plt.axis('equal')
plt.show()

