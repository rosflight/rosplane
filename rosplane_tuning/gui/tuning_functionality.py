import sys
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import *
from PyQt5.QtWidgets import QWidget
from PyQt5.uic import loadUi
from tuning_gui import Ui_MainWindow
import subprocess

class Window(QMainWindow, Ui_MainWindow):
    def __init__(self, parent: QWidget | None = ..., flags: Qt.WindowFlags | Qt.WindowType = ...) -> None:
        super().__init__(parent, flags)
        self.setupUi(self)
        self.connectSignalSlots()
        
        # Object Attributes that we use, with initializations
        self.tuning_mode = ''
        self.curr_kp = 0.0
        self.curr_kd = 0.0
        self.curr_ki = 0.0

    def connectSignalSlots(self):
        # This is where we define signal slots (callbacks) for when the buttons get clicked
        self.CourseButton.toggled.connect(self.courseButtonCallback)
        self.pitchButton.toggled.connect(self.pitchButtonCallback)
        self.rollButton.toggled.connect(self.rollButtonCallback)
        self.airspeedButton.toggled.connect(self.airspeedButtonCallback)
        self.altitudeButton.toggled.connect(self.altitudeButtonCallback)
        self.runButton.clicked.connect(self.runButtonCallback)
        #self.slider.valueChanged.connect(self.slider_callback)
    
    def courseButtonCallback(self):
        # Set the tuning mode
        self.tuning_mode = 'c'
        # Get the other parameters from ROS
        self.curr_kp = self.get_param_output('p')
        self.curr_kd = self.get_param_output('d')
        self.curr_ki = self.get_param_output('i')
        self.set_sliders(self)

    def rollButtonCallback(self):
        # Set the tuning mode
        self.tuning_mode = 'r'
        # Get the other parameters from ROS
        self.curr_kp = self.get_param_output('p')
        self.curr_kd = self.get_param_output('d')
        self.curr_ki = self.get_param_output('i')

    def pitchButtonCallback(self):
        # Set the tuning mode
        self.tuning_mode = 'p'
        # Get the other parameters from ROS
        self.curr_kp = self.get_param_output('p')
        self.curr_kd = self.get_param_output('d')
        self.curr_ki = self.get_param_output('i')

    def airspeedButtonCallback(self):
        # Set the tuning mode
        self.tuning_mode = 'a_t'
        # Get the other parameters from ROS
        self.curr_kp = self.get_param_output('p')
        self.curr_kd = self.get_param_output('d')
        self.curr_ki = self.get_param_output('i')

    def altitudeButtonCallback(self):
        # Set the tuning mode
        self.tuning_mode = 'a'
        # Get the other parameters from ROS
        self.curr_kp = self.get_param_output('p')
        self.curr_kd = self.get_param_output('d')
        self.curr_ki = self.get_param_output('i')
    
    def get_param_output(self, param:str) -> float:
        output = subprocess.run(["ros2","param","get","/autopilot",f"{self.tuning_mode}_k{param}"], stdout=subprocess.PIPE)
        output_list = output.stdout.split()
        output_val = output_list[-1]
        return float(output_val)

    def set_sliders(self):
        self.kpSlider.setMinimum(0)
        self.kpSlider.setMaximum(100)
        self.kiSlider.setMinimum(0)
        self.kiSlider.setMaximum(100)
        self.kdSlider.setMinimum(0)
        self.kdSlider.setMaximum(100)

    #slider stuff 
    #self.slider.valueChanged.connect(self.slider_callback)
    #put somewhere else "self.runButton.clicked.connect(self.runButtonCallback)""

    def runButtonCallback(self):
        #call this if run button is pushed
        #execute ros param set functions
        executable1 = ["ros2", "param", "set", "/autopilot", f"{self.tuning_mode}_kp", f"{self.curr_kp}"]
        executable2 = ["ros2", "param", "set", "/autopilot", f"{self.tuning_mode}_ki", f"{self.curr_ki}"]
        executable3 = ["ros2", "param", "set", "/autopilot", f"{self.tuning_mode}_kd", f"{self.curr_kd}"]
        executables = [executable1, executable2, executable3]
        for executable in executables:
            subprocess.run(executable)