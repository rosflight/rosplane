from optimizer import Optimizer

import numpy as np
import matplotlib.pyplot as plt


def FakeRollController(x, include_noise):
    # Function that roughly mimics the roll controller landscape
    std = 0.1 if include_noise else 0.0
    rotation = np.deg2rad(-40)
    x_trans = x[0] - 0.2
    y_trans = x[1] - 0.03
    x_rot = x_trans * np.cos(rotation) + y_trans * np.sin(rotation)
    y_rot = x_trans * -np.sin(rotation) + y_trans * np.cos(rotation)
    return 15*x_rot**2 + 3*y_rot**2 + np.random.normal(0, std)

# Initialize optimizer
curr_points = np.array([[0.06, 0.04]])  # Initial point
optimization_params = {'mu_1': 1e-4,
                       'mu_2': 0.5,
                       'sigma': 1.5,
                       'alpha': 0.05,
                       'tau': 0.1,
                       'h': 0.05}
optimizer = Optimizer(curr_points[0], optimization_params)
num_repeats = 5

# Print initial point and value
print('Initial point: {}'.format(curr_points[0]))
print('Initial value: {}'.format(FakeRollController(curr_points[0], False)))

# Run optimization
all_points = []
x_0_points = []
while not optimizer.optimization_terminated():
    # Print status
    print(optimizer.get_optimization_status())

    # Calculate error for current points
    error = []
    for point in curr_points:
        error.append(np.average([FakeRollController(point, True) for _ in range(num_repeats)]))
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
print('Total number of points tested: {}'.format(len(all_points) * num_repeats))
print('Final point: {}'.format(curr_points[0]))
print('Final value: {}'.format(FakeRollController(curr_points[0], False)))


# Plot the function with the optimization path
range = 0.5
x = np.linspace(-0.1, 0.5, 100)
y = np.linspace(-0.3, 0.3, 100)
X, Y = np.meshgrid(x, y)
Z = FakeRollController([X, Y], False)
plt.contour(X, Y, Z, 50)
plt.plot(x_0_points[:, 0], x_0_points[:, 1], color='b', marker='o')
# Lock the x and y axis to be equal
plt.axis('equal')
plt.show()

