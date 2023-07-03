import matplotlib
matplotlib.use('qt5agg')
from typing import Optional
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import QMainWindow, QApplication, QWidget, QTabWidget,QVBoxLayout
from scipy.interpolate import interp1d
import sys

class PlotAxis:
    """Used to plot a single axis of data"""
    def __init__(self, ax: plt.Axes, title: str = "", ylabel: str = "", xlabel: str = "time (sec)") -> None:
        """ Initialize the axis

        Inputs:
            ax: Axes on which to plot
            title: Title for the plot
            ylabel: Label for the y-axis of the plot
            xlabel: Label for the x-axis of the plot

        """
        # Store the axis
        self._ax = ax

        # Initialize the axes
        self._ax.set_title(label=title)
        self._ax.set_ylabel(ylabel=ylabel)
        self._ax.set_xlabel(xlabel=xlabel)
        self._ax.autoscale_view(True, True, True)

    def plot(self) -> None:
        """Plots the data. Should be overloaded by children classes to actually plot the data"""
        pass

    def _adjust_axes(self) -> None:
        """Adjusts the axes to have the data in the axes view
        """
        # Resize the axis
        self._ax.relim()
        self._ax.autoscale_view(True, True, True)


class PlotNav(PlotAxis):
    """Used to plot true and navigation data on a single axis"""
    def __init__(self, ax: plt.Axes, true_data: list[float], true_xdata: list[float], nav_data: Optional[list[float]] = None,
                 nav_xdata: Optional[list[float]]=None, cmd_data: Optional[list[float]] = None, cmd_xdata: Optional[list[float]] = None, title: str = "", ylabel: str = "", xlabel: str = "time (sec)",
                 true_label: str = "True", nav_label: str = "Est", cmd_label: str = "Cmd", show_legend: bool = True,
                 nav_color: str = 'lime', true_color: str = 'deepskyblue', cmd_color: str = 'red', widget: Optional[QWidget] = None
                 ) -> None:
        """ Data required for plotting

        Inputs:
            ax: Axes on which to plot
            true_data: True data y-values
            true_xdata: True data x-values (i.e., typically time)
            nav_data: Navigation data y-values
            nav_xdata: Navigation data x-values (i.e., typically time)
            title: Title for the plot
            ylabel: Label for the y-axis of the plot
            xlabel: Label for the x-axis of the plot
            true_label: Label of true data plot in the legend
            nav_label: Label of the nav data plot in the legend
            show_legend: True=> show legend, false => do not
        """
        # Call constructor of super class
        super().__init__(ax=ax, title=title, ylabel=ylabel, xlabel=xlabel)

        # Store the data vectors
        self._true_data = true_data
        self._true_xdata = true_xdata
        self._nav_data = nav_data
        self._nav_xdata = nav_xdata
        self._cmd_data = cmd_data
        self._cmd_xdata = cmd_xdata
        self._widget = widget

        # Create the plots
        self._nav: Optional[Line2D] = None
        if (nav_data is not None) and (nav_xdata is not None):
            (self._nav, ) = self._ax.plot(self._nav_xdata, self._nav_data)
            self._nav.set_label(nav_label)
            self._nav.set_color(nav_color)
            self._nav.set_linewidth(3)
        self._cmd: Optional[Line2D] = None
        if (cmd_data is not None) and (cmd_xdata is not None):
            (self._cmd, ) = self._ax.plot(self._cmd_xdata, self._cmd_data)
            self._cmd.set_label(cmd_label)
            self._cmd.set_color(cmd_color)
            self._cmd.set_linewidth(3)
        (self._true, ) = self._ax.plot(self._true_xdata, self._true_data)
        self._true.set_label(true_label)
        self._true.set_color(true_color)

        # Create the legend
        if show_legend:
            self._ax.legend()

    def plot(self) -> None:
        """Replots the data"""
        # Only plot if the window is active
        # if self._widget is not None and not self._widget.isActiveWindow():
        #     return

        # Replot
        self._true.set_data(self._true_xdata, self._true_data)
        if self._nav is not None:
            self._nav.set_data(self._nav_xdata, self._nav_data)
        if self._cmd is not None:
            self._cmd.set_data(self._cmd_xdata, self._cmd_data)

        # Adjust the axes properties
        self._adjust_axes()

class PlotDiff(PlotAxis):
    """Used to plot the difference between two data sets using interpolation of the data"""
    def __init__(self, ax: plt.Axes, data_y1: list[float], data_x1: list[float], data_y2: list[float],
                 data_x2: list[float], title: str = "", ylabel: str = "", xlabel: str = "time (sec)",
                 legend_label: str = "", show_legend: bool = True,
                 data_color: str = 'red', widget: Optional[QWidget] = None
                 ) -> None:
        """ Data required for plotting

        Inputs:
            ax: Axes on which to plot
            data_y1: Data series 1 y-values
            data_x1: Data series 1 x-values
            data_y2: Data series 2 y-values
            data_x2: Data series 2 x-values
            title: Title for the plot
            ylabel: Label for the y-axis of the plot
            xlabel: Label for the x-axis of the plot
            legend_label: Label for the difference in the legend
            show_legend: True=> show legend, false => do not
        """
        # Call constructor of super class
        super().__init__(ax=ax, title=title, ylabel=ylabel, xlabel=xlabel)

        # Store the data vectors and inputs
        self._data_y1 = data_y1
        self._data_x1 = data_x1
        self._data_y2 = data_y2
        self._data_x2 = data_x2
        self._widget = widget

        # Create the plots
        (self._diff, ) = self._ax.plot(0., 0.)
        self._diff.set_label(legend_label)
        self._diff.set_color(data_color)

        # Create the legend
        if show_legend:
            self._ax.legend()

    def plot(self) -> None:
        """Replots the data"""

        # Only plot if the window is active
        # if self._widget is not None and not self._widget.isActiveWindow():
        #     return

        # Calculate the difference
        y2_interp_fnc = interp1d(x=self._data_x2, y=self._data_y2, kind='linear', bounds_error=False, assume_sorted=True)
        y2_interp = y2_interp_fnc(self._data_x1)
        err = self._data_y1 - y2_interp

        # Replot
        self._diff.set_data(self._data_x1, err)

        # Adjust the axes properties
        self._adjust_axes()


class PlotWindow():
    """ Used to create a tabbed window for plotting. Copied from # Plot window class from https://github.com/superjax/plotWindow
    """
    def __init__(self, parent=None) -> None:
        self.app = QApplication(sys.argv)
        self.MainWindow = QMainWindow()
        self.MainWindow.__init__()
        self.MainWindow.setWindowTitle("Small Unmanned Aerial Vehicle")
        self.canvases = []
        self.figure_handles = []
        self.toolbar_handles = []
        self.tab_handles = []
        self.current_window = -1
        self.tabs = QTabWidget()
        self.MainWindow.setCentralWidget(self.tabs)
        self.MainWindow.resize(1280, 900)
        self.MainWindow.show()

    def addPlot(self, title, figure) -> QWidget:
        new_tab = QWidget()
        layout = QVBoxLayout()
        new_tab.setLayout(layout)

        figure.subplots_adjust(left=0.05, right=0.99, bottom=0.05, top=0.91, wspace=0.2, hspace=0.2)
        new_canvas = FigureCanvas(figure)
        new_toolbar = NavigationToolbar(new_canvas, new_tab)

        layout.addWidget(new_canvas)
        layout.addWidget(new_toolbar)
        self.tabs.addTab(new_tab, title)

        self.toolbar_handles.append(new_toolbar)
        self.canvases.append(new_canvas)
        self.figure_handles.append(figure)
        self.tab_handles.append(new_tab)

        return new_tab

    def draw(self) -> None:
        """Draws the active windows"""

        for (fig, tab) in zip(self.figure_handles, self.tab_handles):
            #if True:# tab.isActiveWindow():
            fig.canvas.draw()
            fig.canvas.flush_events()

    def show(self):
        self.app.exec_()
