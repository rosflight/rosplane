#!/usr/bin/env python3
""" Plots data into a multi-window view
"""

import matplotlib.pyplot as plt
from matplotlib.figure import Figure
import rclpy
from rclpy.executors import SingleThreadedExecutor
import time as pytime
import threading
from data_viz.plot_window import PlotWindow, PlotNav, PlotAxis, PlotDiff
from data_viz.data_storage import RosStorageInterface

class StatePlotter:
    """ Creates a plotting windows and repeatedly plots the data
    """

    def __init__(self, data: RosStorageInterface, plot_sensors: bool, plot_nav_err: bool) -> None:
        """ Create the initial plots, subscriptions, and data storage

        Inputs:
            data: Data storage class for obtaining the data to be plotted
            plot_sensors: True => plot sensors, False => do not
        """
        # Initialize general parameters
        self._plot_sensors = plot_sensors    # True => plot sensor data, False => do not
        self._data = data
        self._sensor_color = "hotpink"

        # Create the plotting window
        self._pw = PlotWindow()
        self._plotters: list[PlotAxis] = []
        self._figures: list[Figure] = []

        # Create the individual figures
        self._create_state_plotter()
        # if plot_nav_err:
        #     self._create_nav_error_plotter()
        # if self._plot_sensors:
        #     self._create_sensor_plotter()

        print("initalized...")

        # Flush out plots and add figures to the GUI
        for fig in self._figures:
            fig.canvas.flush_events()
            #self._pw.addPlot(name, fig)

    def _create_state_plotter(self) -> None:
        """Creates the plot showing the full state of the system
        """
        # Create the inidivual figures
        fig, state_axis = plt.subplots(5, 4)
        self._pw.addPlot("States", fig)
        widget = self._figures.append(fig)

        # Create position plots
        self._plotters.append(PlotNav(
            true_data=self._data.true.pn, true_xdata=self._data.true.time,
            nav_data=self._data.est.pn, nav_xdata=self._data.est.time,
            true_label="$p_n$-True", nav_label="$p_n$-Est", show_legend=True,
            ax=state_axis[0,0], title="x-axis", ylabel="$m$", widget=widget
        ))
        self._plotters.append(PlotNav(
            true_data=self._data.true.pe, true_xdata=self._data.true.time,
            nav_data=self._data.est.pe, nav_xdata=self._data.est.time,
            true_label="$p_e$-True", nav_label="$p_e$-Est", show_legend=True,
            ax=state_axis[0,1], title="y-axis", ylabel="$m$", widget=widget
        ))
        self._plotters.append(PlotNav(
            true_data=self._data.true.p_alt, true_xdata=self._data.true.time,
            nav_data=self._data.est.p_alt, nav_xdata=self._data.est.time,
            true_label="alt-True", nav_label="alt-Est", show_legend=True,
            ax=state_axis[0,2], title="z-axis", ylabel="$m$", widget=widget
        ))
        self._plotters.append(PlotNav(
            true_data=self._data.true.w_n, true_xdata=self._data.true.time,
            nav_data=self._data.true.w_e, nav_xdata=self._data.true.time,
            ax=state_axis[0,3], title="Wind", ylabel="$m/s$",
            true_label="$w_n$", nav_label="$w_e$", show_legend=True,
            true_color="darkorange", nav_color="forestgreen", widget=widget
        ))

        # Create linear velocity plots
        self._plotters.append(PlotNav(
            true_data=self._data.true.u, true_xdata=self._data.true.time,
            nav_data=self._data.est.u, nav_xdata=self._data.est.time,
            true_label="$u$-True", nav_label="$u$-Est", show_legend=True,
            ax=state_axis[1,0], title="", ylabel="$m/s$", widget=widget
        ))
        self._plotters.append(PlotNav(
            true_data=self._data.true.v, true_xdata=self._data.true.time,
            nav_data=self._data.est.v, nav_xdata=self._data.est.time,
            true_label="$v$-True", nav_label="$v$-Est", show_legend=True,
            ax=state_axis[1,1], title="", ylabel="$m/s$", widget=widget
        ))
        self._plotters.append(PlotNav(
            true_data=self._data.true.w, true_xdata=self._data.true.time,
            nav_data=self._data.est.w, nav_xdata=self._data.est.time,
            true_label="$w$-True", nav_label="$w$-Est", show_legend=True,
            ax=state_axis[1,2], title="", ylabel="$m/s$", widget=widget
        ))
        self._plotters.append(PlotNav(
            true_data=self._data.true.alpha, true_xdata=self._data.true.time,
            nav_data=self._data.true.beta, nav_xdata=self._data.true.time,
            ax=state_axis[1,3], title="", ylabel="rad",
            true_label="$\\alpha $", nav_label="$\\beta$", show_legend=False,
            true_color="darkorange", nav_color="forestgreen", widget=widget
        ))
        self._plotters.append(PlotNav(
            true_data=self._data.true.chi, true_xdata=self._data.true.time,
            ax=state_axis[1,3], title="", ylabel="rad",
            true_label="$\\chi$", show_legend=True,
            true_color="sienna", widget=widget
        ))

        # Create attitude plots
        self._plotters.append(PlotNav(
            true_data=self._data.true.phi, true_xdata=self._data.true.time,
            nav_data=self._data.est.phi, nav_xdata=self._data.est.time,
            true_label="$\\phi$-True", nav_label="$\\phi$-Est", show_legend=True,
            ax=state_axis[2,0], title="", ylabel="$rad$", widget=widget
        ))
        self._plotters.append(PlotNav(
            true_data=self._data.true.theta, true_xdata=self._data.true.time,
            nav_data=self._data.est.theta, nav_xdata=self._data.est.time,
            true_label="$\\theta$-True", nav_label="$\\theta$-Est", show_legend=True,
            ax=state_axis[2,1], title="", ylabel="$rad$", widget=widget
        ))
        self._plotters.append(PlotNav(
            true_data=self._data.true.psi, true_xdata=self._data.true.time,
            nav_data=self._data.est.psi, nav_xdata=self._data.est.time,
            true_label="$\\psi$-True", nav_label="$\\psi$-Est", show_legend=True,
            ax=state_axis[2,2], title="", ylabel="$rad$", widget=widget
        ))
        self._plotters.append(PlotNav(
            true_data=self._data.true.v_a, true_xdata=self._data.true.time,
            nav_data=self._data.est.v_a, nav_xdata=self._data.est.time,
            true_label="$V_a$-True", nav_label="$V_a$-Est", show_legend=True,
            ax=state_axis[2,3], title="", ylabel="$m/s$", widget=widget
        ))

        # Create angular velocity plots
        self._plotters.append(PlotNav(
            true_data=self._data.true.p, true_xdata=self._data.true.time,
            nav_data=self._data.est.p, nav_xdata=self._data.est.time,
            true_label="$p$-True", nav_label = "$p$-Est", show_legend=True,
            ax=state_axis[3,0], title="", ylabel="$rad/s$", widget=widget
        ))
        self._plotters.append(PlotNav(
            true_data=self._data.true.q, true_xdata=self._data.true.time,
            nav_data=self._data.est.q, nav_xdata=self._data.est.time,
            true_label="$q$-True", nav_label="$q$-Est", show_legend=True,
            ax=state_axis[3,1], title="", ylabel="$rad/s$", widget=widget
        ))
        self._plotters.append(PlotNav(
            true_data=self._data.true.r, true_xdata=self._data.true.time,
            nav_data=self._data.est.r, nav_xdata=self._data.est.time,
            true_label="$r$-True", nav_label="$r$-Est", show_legend=True,
            ax=state_axis[3,2], title="", ylabel="$rad/s$", widget=widget
        ))
        self._plotters.append(PlotNav(
            true_data=self._data.true.v_g, true_xdata=self._data.true.time,
            true_label="$V_g$-True", show_legend=True,
            ax=state_axis[3,3], title="", ylabel="$m/s$", widget=widget
        ))


        # Create control plots
        self._plotters.append(PlotNav(
            true_data=self._data.cmd.elevator, true_xdata=self._data.cmd.time,
            true_label="$\\delta_e$", show_legend=True,
            ax=state_axis[4,0], title="", ylabel="", widget=widget
        ))
        self._plotters.append(PlotNav(
            true_data=self._data.cmd.aileron, true_xdata=self._data.cmd.time,
            true_label="$\\delta_a$", show_legend=True,
            ax=state_axis[4,1], title="", ylabel="", widget=widget
        ))
        self._plotters.append(PlotNav(
            true_data=self._data.cmd.rudder, true_xdata=self._data.cmd.time,
            true_label="$\\delta_r$", show_legend=True,
            ax=state_axis[4,2], title="", ylabel="", widget=widget
        ))
        self._plotters.append(PlotNav(
            true_data=self._data.cmd.throttle, true_xdata=self._data.cmd.time,
            true_label="$\\delta_t$", show_legend=True,
            ax=state_axis[4,3], title="", ylabel="", widget=widget
        ))

    # def _create_nav_error_plotter(self) -> None:
    #     """Creates the plotting windows for navigation error
    #     """
    #     # Create the inidivual figures
    #     fig, nav_axis = plt.subplots(5, 4)
    #     widget = self._pw.addPlot("Nav Error", fig)
    #     self._figures.append(fig)
    #
    #     # Create position plots
    #     self._plotters.append(PlotDiff(
    #         data_y1=self._data.true.pn, data_x1=self._data.true.time,
    #         data_y2=self._data.est.pn, data_x2=self._data.est.time,
    #         legend_label="$\\delta p_n$", show_legend=True,
    #         ax=nav_axis[0,0], title="", ylabel="$m$", widget=widget
    #     ))
    #     self._plotters.append(PlotDiff(
    #         data_y1=self._data.true.pe, data_x1=self._data.true.time,
    #         data_y2=self._data.est.pe, data_x2=self._data.est.time,
    #         legend_label="$\\delta p_e$", show_legend=True,
    #         ax=nav_axis[0,1], title="", ylabel="$m$", widget=widget
    #     ))
    #     self._plotters.append(PlotDiff(
    #         data_y1=self._data.true.p_alt, data_x1=self._data.true.time,
    #         data_y2=self._data.est.p_alt, data_x2=self._data.est.time,
    #         legend_label="$\\delta p_{alt}$", show_legend=True,
    #         ax=nav_axis[0,2], title="", ylabel="$m$", widget=widget
    #     ))
    #
    #     # Create linear velocity plots
    #     self._plotters.append(PlotDiff(
    #         data_y1=self._data.true.u, data_x1=self._data.true.time,
    #         data_y2=self._data.est.u, data_x2=self._data.est.time,
    #         legend_label="$\\delta u$", show_legend=True,
    #         ax=nav_axis[1,0], title="", ylabel="$m/s$", widget=widget
    #     ))
    #     self._plotters.append(PlotDiff(
    #         data_y1=self._data.true.v, data_x1=self._data.true.time,
    #         data_y2=self._data.est.v, data_x2=self._data.est.time,
    #         legend_label="$\\delta v$", show_legend=True,
    #         ax=nav_axis[1,1], title="", ylabel="$m/s$", widget=widget
    #     ))
    #     self._plotters.append(PlotDiff(
    #         data_y1=self._data.true.w, data_x1=self._data.true.time,
    #         data_y2=self._data.est.w, data_x2=self._data.est.time,
    #         legend_label="$\\delta w$", show_legend=True,
    #         ax=nav_axis[1,2], title="", ylabel="$m/s$", widget=widget
    #     ))
    #
    #     # Create the airspeed plots
    #     self._plotters.append(PlotDiff(
    #         data_y1=self._data.true.v_a, data_x1=self._data.true.time,
    #         data_y2=self._data.est.v_a, data_x2=self._data.est.time,
    #         legend_label="$\\delta v_a$", show_legend=True, data_color='red',
    #         ax=nav_axis[1,3], title="", ylabel="$m/s$", widget=widget
    #     ))
    #     self._plotters.append(PlotDiff(
    #         data_y1=self._data.true.v_g, data_x1=self._data.true.time,
    #         data_y2=self._data.est.v_g, data_x2=self._data.est.time,
    #         legend_label="$\\delta v_g$", show_legend=True, data_color='blue',
    #         ax=nav_axis[1,3], title="", ylabel="$m/s$", widget=widget
    #     ))
    #
    #     # Create attitude plots
    #     self._plotters.append(PlotDiff(
    #         data_y1=self._data.true.phi, data_x1=self._data.true.time,
    #         data_y2=self._data.est.phi, data_x2=self._data.est.time,
    #         legend_label="$\\delta \\phi$", show_legend=True,
    #         ax=nav_axis[2,0], title="", ylabel="$rad$", widget=widget
    #     ))
    #     self._plotters.append(PlotDiff(
    #         data_y1=self._data.true.theta, data_x1=self._data.true.time,
    #         data_y2=self._data.est.theta, data_x2=self._data.est.time,
    #         legend_label="$\\delta \\theta$", show_legend=True,
    #         ax=nav_axis[2,1], title="", ylabel="$rad$", widget=widget
    #     ))
    #     self._plotters.append(PlotDiff(
    #         data_y1=self._data.true.psi, data_x1=self._data.true.time,
    #         data_y2=self._data.est.psi, data_x2=self._data.est.time,
    #         legend_label="$\\delta \\psi$", show_legend=True,
    #         ax=nav_axis[2,2], title="", ylabel="$rad$", widget=widget
    #     ))
    #
    #     # Create angular velocity plots
    #     self._plotters.append(PlotDiff(
    #         data_y1=self._data.true.p, data_x1=self._data.true.time,
    #         data_y2=self._data.est.p, data_x2=self._data.est.time,
    #         legend_label="$\\delta p$", show_legend=True,
    #         ax=nav_axis[3,0], title="", ylabel="$rad/s$", widget=widget
    #     ))
    #     self._plotters.append(PlotDiff(
    #         data_y1=self._data.true.q, data_x1=self._data.true.time,
    #         data_y2=self._data.est.q, data_x2=self._data.est.time,
    #         legend_label="$\\delta q$", show_legend=True,
    #         ax=nav_axis[3,1], title="", ylabel="$rad/s$", widget=widget
    #     ))
    #     self._plotters.append(PlotDiff(
    #         data_y1=self._data.true.r, data_x1=self._data.true.time,
    #         data_y2=self._data.est.r, data_x2=self._data.est.time,
    #         legend_label="$\\delta r$", show_legend=True,
    #         ax=nav_axis[3,2], title="", ylabel="$rad/s$", widget=widget
    #     ))
    #
    #     # Create the gyro bias plots
    #     self._plotters.append(PlotDiff(
    #         data_y1=self._data.true.gyro_bx, data_x1=self._data.true.time,
    #         data_y2=self._data.est.gyro_bx, data_x2=self._data.est.time,
    #         legend_label="gyro-$\\delta b_x$", show_legend=True, data_color='red',
    #         ax=nav_axis[3,3], title="", ylabel="$rad/s$", widget=widget
    #     ))
    #     self._plotters.append(PlotDiff(
    #         data_y1=self._data.true.gyro_by, data_x1=self._data.true.time,
    #         data_y2=self._data.est.gyro_by, data_x2=self._data.est.time,
    #         legend_label="gyro-$\\delta b_y$", show_legend=True, data_color='blue',
    #         ax=nav_axis[3,3], title="", ylabel="$rad/s$", widget=widget
    #     ))
    #     self._plotters.append(PlotDiff(
    #         data_y1=self._data.true.gyro_bz, data_x1=self._data.true.time,
    #         data_y2=self._data.est.gyro_bz, data_x2=self._data.est.time,
    #         legend_label="gyro-$\\delta b_z$", show_legend=True, data_color='lime',
    #         ax=nav_axis[3,3], title="", ylabel="$rad/s$", widget=widget
    #     ))

    # def _create_sensor_plotter(self) -> None:
    #     """Creates the plotting window for sensor data
    #     """
    #     # Create the inidivual figures
    #     fig, sense_axis = plt.subplots(4, 3)
    #     widget = self._pw.addPlot("Sensors", fig)
    #     self._figures.append(fig)
    #
    #     # Create the gps plots
    #     self._plotters.append(PlotNav(
    #         true_data=self._data.gps.pn, true_xdata=self._data.gps.time,
    #         true_label="$p_n$-GPS", true_color=self._sensor_color, show_legend=True,
    #         ax=sense_axis[0,0], title="", ylabel="$m$", widget=widget
    #     ))
    #     self._plotters.append(PlotNav(
    #         true_data=self._data.gps.pe, true_xdata=self._data.gps.time,
    #         true_label="$p_e$-GPS", true_color=self._sensor_color, show_legend=True,
    #         ax=sense_axis[0,1], title="", ylabel="$m$", widget=widget
    #     ))
    #     self._plotters.append(PlotNav(
    #         true_data=self._data.gps.p_alt, true_xdata=self._data.gps.time,
    #         true_label="alt-GPS", true_color=self._sensor_color, show_legend=True,
    #         ax=sense_axis[0,2], title="", ylabel="$m$", widget=widget
    #     ))
    #
    #     # GPS velocity, compass, and pressure sensors
    #     self._plotters.append(PlotNav(
    #         true_data=self._data.gps.v_g, true_xdata=self._data.gps.time,
    #         true_label="$V_g$-GPS", true_color=self._sensor_color, show_legend=True,
    #         ax=sense_axis[1,0], title="", ylabel="$m/s$", widget=widget
    #     ))
    #     self._plotters.append(PlotNav(
    #         true_data=self._data.comp.angle, true_xdata=self._data.comp.time,
    #         true_color=self._sensor_color, true_label="$\\psi$-Compass", show_legend=True,
    #         ax=sense_axis[1,1], title="", ylabel="$rad$", widget=widget
    #     ))
    #     self._plotters.append(PlotNav(
    #         true_data=self._data.true.psi, true_xdata=self._data.true.time,
    #         nav_data=self._data.est.psi, nav_xdata=self._data.est.time,
    #         true_label="$\\psi$-True", nav_label="$\\psi$-Est", show_legend=True,
    #         ax=sense_axis[1,1], title="", ylabel="$rad$", widget=widget
    #     ))
    #     self._plotters.append(PlotNav(
    #         true_data=self._data.press.abs_pressure, true_xdata=self._data.press.time,
    #         nav_data=self._data.press.diff_pressure, nav_xdata=self._data.press.time,
    #         nav_color=self._sensor_color, true_color="darkviolet",
    #         true_label="abs-press", nav_label="diff-press", show_legend=True,
    #         ax=sense_axis[1,2], title="", ylabel="pascals", widget=widget
    #     ))
    #
    #     # Create the accelerometer plots
    #     self._plotters.append(PlotNav(
    #             true_data=self._data.imu.ax, true_xdata=self._data.imu.time,
    #             true_label="accel-x", true_color=self._sensor_color, show_legend=True,
    #             ax=sense_axis[2,0], title="", ylabel="$m/s^2$", widget=widget
    #         )
    #     )
    #     self._plotters.append(PlotNav(
    #             true_data=self._data.imu.ay, true_xdata=self._data.imu.time,
    #             true_label="accel-y", true_color=self._sensor_color, show_legend=True,
    #             ax=sense_axis[2,1], title="", ylabel="$m/s^2$", widget=widget
    #         )
    #     )
    #     self._plotters.append(PlotNav(
    #             true_data=self._data.imu.az, true_xdata=self._data.imu.time,
    #             true_label="accel-z", true_color=self._sensor_color, show_legend=True,
    #             ax=sense_axis[2,2], title="", ylabel="$m/s^2$", widget=widget
    #         )
    #     )
    #
    #     # Create the gyro plots
    #     self._plotters.append(PlotNav(
    #             true_data=self._data.imu.p, true_xdata=self._data.imu.time,
    #             true_label="gyro-x", true_color=self._sensor_color, show_legend=True,
    #             ax=sense_axis[3,0], title="", ylabel="rad/s", widget=widget
    #         )
    #     )
    #     self._plotters.append(PlotNav(
    #             true_data=self._data.true.gyro_bx, true_xdata=self._data.true.time,
    #             nav_data=self._data.est.gyro_bx, nav_xdata=self._data.est.time,
    #             true_label="True-bias", nav_label="Nav-bias", show_legend=True,
    #             ax=sense_axis[3,0], title="", ylabel="rad/s", widget=widget
    #         )
    #     )
    #     self._plotters.append(PlotNav(
    #             true_data=self._data.imu.q, true_xdata=self._data.imu.time,
    #             true_label="gyro-y", true_color=self._sensor_color, show_legend=True,
    #             ax=sense_axis[3,1], title="", ylabel="rad/s", widget=widget
    #         )
    #     )
    #     self._plotters.append(PlotNav(
    #             true_data=self._data.true.gyro_by, true_xdata=self._data.true.time,
    #             nav_data=self._data.est.gyro_by, nav_xdata=self._data.est.time,
    #             true_label="True-bias", nav_label="Nav-bias", show_legend=True,
    #             ax=sense_axis[3,1], title="", ylabel="rad/s", widget=widget
    #         )
    #     )
    #     self._plotters.append(PlotNav(
    #             true_data=self._data.imu.r, true_xdata=self._data.imu.time,
    #             true_label="gyro-z", true_color=self._sensor_color, show_legend=True,
    #             ax=sense_axis[3,2], title="", ylabel="rad/s", widget=widget
    #         )
    #     )
    #     self._plotters.append(PlotNav(
    #             true_data=self._data.true.gyro_bz, true_xdata=self._data.true.time,
    #             nav_data=self._data.est.gyro_bz, nav_xdata=self._data.est.time,
    #             true_label="True-bias", nav_label="Nav-bias", show_legend=True,
    #             ax=sense_axis[3,2], title="", ylabel="rad/s", widget=widget
    #         )
    #     )

    def plot_states(self) -> None:
        """Plots the state updates"""
        try:
            with self._data.lock:
                for plotter in self._plotters:
                    plotter.plot()

            # Draw the figure
            self._pw.draw()

        except Exception as e:
            self._data.node.get_logger().error("Error in plotting: " + str(e))

def main(args=None):
    # Initialize the node and read in parameters
    rclpy.init(args=args)
    node = rclpy.create_node(node_name="plotter_node")
    node.declare_parameter("t_horizon", 100.)
    node.declare_parameter("plot_sensors", False)
    node.declare_parameter("plot_nav_error", False)
    t_horizon: float = node.get_parameter("t_horizon").value
    plot_sensors: bool = node.get_parameter("plot_sensors").value
    plot_nav_err: bool = node.get_parameter("plot_nav_error").value

    # Create the state plotter
    data = RosStorageInterface(node=node, t_horizon=t_horizon)
    plotter = StatePlotter(data=data, plot_sensors=plot_sensors, plot_nav_err=plot_nav_err)

    # Create the executor
    exec = SingleThreadedExecutor()
    exec.add_node(node)
    thread = threading.Thread(target=exec.spin, daemon=True)
    thread.start()

    while rclpy.ok():
        plotter.plot_states()
        pytime.sleep(1.0) # Sleep

    thread.join()

if __name__ == '__main__':
    main()