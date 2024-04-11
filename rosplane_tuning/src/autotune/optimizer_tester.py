from optimizer import Optimizer

import numpy as np
import matplotlib.pyplot as plt


def FakeRollController(x, include_noise):
    # Function that roughly mimics the roll controller landscape
    std = 0.1 if include_noise else 0.0

    rotation = np.deg2rad(35)
    x_offset = 0.20
    y_offset = 0.03
    x_scale = 15
    y_scale = 3

    x_rot = x[0] * np.cos(rotation) - x[1] * np.sin(rotation)
    y_rot = x[0] * np.sin(rotation) + x[1] * np.cos(rotation)
    x_trans = x_rot - (x_offset*np.cos(rotation) - y_offset*np.sin(rotation))
    y_trans = y_rot - (x_offset*np.sin(rotation) + y_offset*np.cos(rotation))
    x_scaled = x_trans**2 * x_scale
    y_scaled = y_trans**2 * y_scale
    return x_scaled + y_scaled + np.random.normal(0, std)

# Initialize optimizer
curr_points = np.array([[0.06, 0.04]])  # Initial point
optimization_params = {'alpha': 0.1,
                       'tau': 0.1,
                       'h': 0.03}
optimizer = Optimizer(curr_points[0], optimization_params)

# Print initial point and value
print('Initial point: {}'.format(curr_points[0]))
print('Initial value: {}'.format(FakeRollController(curr_points[0], False)))

# Run optimization
all_points = []
x_points = []
while not optimizer.optimization_terminated():
    # Calculate error for current points
    error = []
    for point in curr_points:
        error.append(FakeRollController(point, True))
    error = np.array(error)
    # Pass points to optimizer
    curr_points = optimizer.get_next_parameter_set(error)

    # Store points 
    for point in curr_points:
        all_points.append(point)
    x_points.append(optimizer.x)

    # End interation step
all_points = np.array(all_points)
x_points = np.array(x_points)

print('Optimization terminated with status: {}'.format(optimizer.get_optimization_status()))
print('Final value: {}'.format(FakeRollController(curr_points[0], False)))
print('Total number of points tested: {}'.format(len(all_points)))


# Plot the function with the optimization path
x = np.linspace(0., 0.4, 100)
y = np.linspace(-0.2, 0.3, 100)
X, Y = np.meshgrid(x, y)
Z = FakeRollController([X, Y], False)
plt.contour(X, Y, Z, 50)
plt.plot(x_points[:, 0], x_points[:, 1], color='b', marker='o')
# Lock the x and y axis to be equal
plt.axis('equal')
plt.show()

