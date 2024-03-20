import sys
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import *
from PyQt5.QtWidgets import QWidget
from PyQt5.uic import loadUi
from tuning_gui import Ui_MainWindow
import subprocess
import timeit

class Window(QMainWindow, Ui_MainWindow):
    def __init__(self): #, parent: QWidget | None = ..., flags: Qt.WindowFlags | Qt.WindowType = ...) -> None:
        super().__init__() #parent, flags)
        self.setupUi(self)
        self.connectSignalSlots()
        
        # Object Attributes that we use, with initializations
        self.tuning_mode = ''
        self.curr_kp = 0.0
        self.curr_kd = 0.0
        self.curr_ki = 0.0
        self.initialize_temps()
        # This allows us to have different ranges for fine tuning kp, ki, and kd
        self.kp_edit_dist = 2.0
        self.ki_edit_dist = 0.5
        self.kd_edit_dist = 2.0
        # Boolean values for controlling debugging statements
        self.time = False
        self.disp = True
    
    def initialize_temps(self):
        self.temp_kp = 0.0
        self.temp_kd = 0.0
        self.temp_ki = 0.0

    def connectSignalSlots(self):
        # This is where we define signal slots (callbacks) for when the buttons get clicked
        self.CourseButton.toggled.connect(self.courseButtonCallback)
        self.pitchButton.toggled.connect(self.pitchButtonCallback)
        self.rollButton.toggled.connect(self.rollButtonCallback)
        self.airspeedButton.toggled.connect(self.airspeedButtonCallback)
        self.altitudeButton.toggled.connect(self.altitudeButtonCallback)
        self.runButton.clicked.connect(self.runButtonCallback)
        self.kpSlider.sliderReleased.connect(self.kp_slider_callback)
        self.kiSlider.sliderReleased.connect(self.ki_slider_callback)
        self.kdSlider.sliderReleased.connect(self.kd_slider_callback)
    
    def courseButtonCallback(self):
        # Set the tuning mode
        self.tuning_mode = 'c'
        # Get the other parameters from ROS
        self.curr_kp = self.get_param_output('p')
        self.curr_kd = self.get_param_output('d')
        self.curr_ki = self.get_param_output('i')
        # Set the sliders to the appropriate values
        self.set_sliders()
        self.initialize_temps()

    def rollButtonCallback(self):
        # Set the tuning mode
        self.tuning_mode = 'r'
        # Get the other parameters from ROS
        self.curr_kp = self.get_param_output('p')
        self.curr_kd = self.get_param_output('d')
        self.curr_ki = self.get_param_output('i')
        self.set_sliders()
        self.initialize_temps()

    def pitchButtonCallback(self):
        # Set the tuning mode
        self.tuning_mode = 'p'
        # Get the other parameters from ROS
        self.curr_kp = self.get_param_output('p')
        self.curr_kd = self.get_param_output('d')
        self.curr_ki = self.get_param_output('i')
        self.set_sliders()
        self.initialize_temps()

    def airspeedButtonCallback(self):
        # Set the tuning mode
        self.tuning_mode = 'a_t'
        # Get the other parameters from ROS
        self.curr_kp = self.get_param_output('p')
        self.curr_kd = self.get_param_output('d')
        self.curr_ki = self.get_param_output('i')
        self.set_sliders()
        self.initialize_temps()

    def altitudeButtonCallback(self):
        # Set the tuning mode
        self.tuning_mode = 'a'
        # Get the other parameters from ROS
        self.curr_kp = self.get_param_output('p')
        self.curr_kd = self.get_param_output('d')
        self.curr_ki = self.get_param_output('i')
        self.set_sliders()
        self.initialize_temps()
    
    def get_param_output(self, param:str) -> float:
        if self.time: start = timeit.timeit()
        output = subprocess.run(["ros2","param","get","/autopilot",f"{self.tuning_mode}_k{param}"], stdout=subprocess.PIPE)
        try:
            output_list = output.stdout.split()
            output_val = output_list[-1]
            if self.time: print(timeit.timeit() - start)
            return float(output_val)
        except IndexError:
            print("Error accessing ROS service output! Output:", output.stdout)
        return -1

    def set_sliders(self):
        # Sliders have an integer range. Set this from +- 100
        self.kpSlider.setValue(0)
        self.kpSlider.setMinimum(-100)
        self.kpSlider.setMaximum(100)

        self.kiSlider.setValue(0)
        self.kiSlider.setMinimum(-100)
        self.kiSlider.setMaximum(100)

        self.kdSlider.setValue(0)
        self.kdSlider.setMinimum(-100)
        self.kdSlider.setMaximum(100)
    
    def kp_slider_callback(self):
        slider_val = self.kpSlider.value()
        self.temp_kp = self.curr_kp + self.kp_edit_dist * slider_val / 100
        if self.disp: print(self.temp_kp)
    
    def ki_slider_callback(self):
        slider_val = self.kiSlider.value()
        self.temp_ki = self.curr_ki + self.ki_edit_dist * slider_val / 100
        if self.disp: print(self.temp_ki)
   
    def kd_slider_callback(self):
        slider_val = self.kdSlider.value()
        self.temp_kd = self.curr_kd + self.kd_edit_dist * slider_val / 100
        if self.disp: print(slider_val, self.temp_kd)

    #slider stuff 
    #self.slider.valueChanged.connect(self.slider_callback)
    #put somewhere else "self.runButton.clicked.connect(self.runButtonCallback)""

    def runButtonCallback(self):
        #call this if run button is pushed
        # Set current variables to be temp variables
        self.curr_kp = self.temp_kp
        self.curr_ki = self.temp_ki
        self.curr_kd = self.temp_kd
        #execute ros param set functions
        executable1 = ["ros2", "param", "set", "/autopilot", f"{self.tuning_mode}_kp", f"{self.curr_kp}"]
        executable2 = ["ros2", "param", "set", "/autopilot", f"{self.tuning_mode}_ki", f"{self.curr_ki}"]
        executable3 = ["ros2", "param", "set", "/autopilot", f"{self.tuning_mode}_kd", f"{self.curr_kd}"]
        executables = [executable1, executable2, executable3]
        for executable in executables:
            output = subprocess.run(executable, stdout=subprocess.PIPE)
            if self.disp: print(output.stdout)
        if self.disp:
            print('Kp set to:', self.curr_kp)
            print('Ki set to:', self.curr_ki)
            print('Kd set to:', self.curr_kd)


# Main loop
if __name__=='__main__':
    app = QApplication(sys.argv)
    window = Window()
    window.show()
    sys.exit(app.exec_())
