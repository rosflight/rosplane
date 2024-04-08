from optimizer import Optimizer

import numpy as np
import matplotlib.pyplot as plt


# Function to test the optimizer with
def function(x):
    # Matyas function
    print("f", x)
    return 0.26 * (x[0] ** 2 + x[1] ** 2) - 0.48 * x[0] * x[1]

def gradient(x):
    # Gradient of Matyas function
    return np.array([0.52 * x[0] - 0.48 * x[1], 0.52 * x[1] - 0.48 * x[0]])

# Initialize optimizer
curr_points = np.array([[0.0, 5.0]])  # Initial point
optimization_params = {'u1': 1e-4,
                       'u2': 0.5,
                       'sigma': 1.5,
                       'alpha': 1.0,
                       'tau': 1e-3}
optimizer = Optimizer(curr_points[0], optimization_params)


# Run optimization
all_points = []
k = 0
while not optimizer.optimization_terminated():
    print("Iteration ", k)
    # Print status
    print(optimizer.get_optimiztion_status())
    print(optimizer.state)

    # Calculate error for current points
    error = []
    print("CP", curr_points)                      # Testing
    for point in curr_points:
        error.append(function(point))
    error = np.array(error)
    # Pass points to optimizer
    # print("CP", curr_points)                  # Testing
    # print("G", gradient(curr_points[0]))    # Testing
    curr_points = optimizer.get_next_parameter_set(error)
    # print("CP", curr_points)                  # Testing

    # Store points 
    for point in curr_points:
        all_points.append(point)

    # End interation step
    k += 1
    print()
all_points = np.array(all_points)

print('Optimization terminated with status: {}'.format(optimizer.get_optimiztion_status()))


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
plt.plot(all_points[:, 0], all_points[:, 1], marker='o', color='r')
plt.show()

