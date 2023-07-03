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

        # print("initalized...")

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
            nav_data=self._data.est.p_alt, nav_xdata=self._data.est.time, cmd_data=self._data.cmd_state.altitude,
            cmd_xdata=self._data.cmd_state.time, cmd_label="$alt_{cmd}$",
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
            true_data=self._data.true.chi, true_xdata=self._data.true.time, cmd_data=self._data.cmd_state.course,
            cmd_xdata=self._data.cmd_state.time, ax=state_axis[1,3], title="", ylabel="rad",
            true_label="$\\chi$", cmd_label="$\\chi_{cmd}$", show_legend=True,
            true_color="sienna", widget=widget
        ))

        # Create attitude plots
        self._plotters.append(PlotNav(
            true_data=self._data.true.phi, true_xdata=self._data.true.time,
            nav_data=self._data.est.phi, nav_xdata=self._data.est.time, cmd_data=self._data.cmd_state.roll,
            cmd_xdata=self._data.cmd_state.time, true_label="$\\phi$-True", nav_label="$\\phi$-Est",
            cmd_label="$\\phi_{cmd}$", show_legend=True, ax=state_axis[2,0], title="", ylabel="$rad$", widget=widget
        ))
        self._plotters.append(PlotNav(
            true_data=self._data.true.theta, true_xdata=self._data.true.time,
            nav_data=self._data.est.theta, nav_xdata=self._data.est.time,  cmd_data=self._data.cmd_state.pitch,
            cmd_xdata=self._data.cmd_state.time, true_label="$\\theta$-True",
            nav_label="$\\theta$-Est", show_legend=True, cmd_label="$\\theta_{cmd}$",
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