#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

class Optimizer:
    """
    This class is a utility that can be used to plot the function landscape of the autopilot
    being tuned. It acts as a dummy optimizer that instead of running an optimization, it collects
    data based on a grid of points and then plots the function landscape.

    WARNING: Plotting is intended for simulation only, do not attempt to plot on real hardware
    as the autopilot may be given non-functional gains. As such, no launch files are provided for
    this utility. If you wish to use it, modify the source code of the autotune node to
    include this class instead of the optimizer.
    """

    def __init__(self, *args):
        # None of the optimization parameters are needed, so they are ignored.

        # Define the grid of points to test and plot
        self.x_values = np.linspace(0.0, 1.0, 5)
        self.y_values = np.linspace(0.0, 1.0, 5)

        self.points_given = False
        self.plotting_complete = False

    def optimization_terminated(self):
        return self.plotting_complete

    def get_optimization_status(self):
        if self.plotting_complete:
            return "Plotting complete."
        else:
            return "Collecting data for plotting."

    def get_next_parameter_set(self, error):
        if not self.points_given:
            self.points_given = True
            x, y = np.meshgrid(self.x_values, self.y_values)
            return np.column_stack((x.ravel(), y.ravel()))
        else:
            self.plot(error)
            self.plotting_complete = True
            return np.zeros((1, 2))

    def plot(self, error):
        x, y = np.meshgrid(self.x_values, self.y_values)
        z = error.reshape(x.shape)

        # Plot the function landscape in a 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot_surface(x, y, z, cmap='viridis', alpha=0.8)
        ax.set_xlabel('Gain 1')
        ax.set_ylabel('Gain 2')
        ax.set_zlabel('Error')
        plt.show()

        # Save to npy file, iterate the filename if it already exists
        xyz = np.stack((x, y, z), axis=-1)
        file_index = 0
        while True:
            filename = f'plot_data_{file_index}.npy'
            try:
                with open(filename, 'xb') as f:  # Use 'xb' mode to create the file exclusively
                    np.save(f, xyz)
                break  # Exit loop if file creation is successful
            except FileExistsError:
                file_index += 1

