import sys
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import *
from PyQt5.QtWidgets import QWidget
from PyQt5.uic import loadUi
from tuning_gui import Ui_MainWindow
import subprocess
import timeit

# TODO:
# 1. Add the clear button
# 2. Add the undo button
# 3. Make the GUI prettier (expanding frames, etc.)
# 4. Why is it so slow sometimes?

class Window(QMainWindow, Ui_MainWindow):
    def __init__(self): 
        super().__init__() 
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
        self.kpSlider.valueChanged.connect(self.kp_slider_callback)
        self.kiSlider.valueChanged.connect(self.ki_slider_callback)
        self.kdSlider.valueChanged.connect(self.kd_slider_callback)
        self.kpSpinBox.valueChanged.connect(self.kpSpinBox_callback)
        self.kiSpinBox.valueChanged.connect(self.kiSpinBox_callback)
        self.kdSpinBox.valueChanged.connect(self.kdSpinBox_callback)
    
    def courseButtonCallback(self):
        if self.CourseButton.isChecked():
            # Set the tuning mode
            self.tuning_mode = 'c'
            # Get the other parameters from ROS
            self.curr_kp = self.get_param_output('p')
            self.curr_kd = self.get_param_output('d')
            self.curr_ki = self.get_param_output('i')
            # Set the sliders to the appropriate values
            self.set_sliders()
            self.set_SpinBoxes()
            self.initialize_temps()

    def rollButtonCallback(self):
        if self.rollButton.isChecked():
            # Set the tuning mode
            self.tuning_mode = 'r'
            # Get the other parameters from ROS
            self.curr_kp = self.get_param_output('p')
            self.curr_kd = self.get_param_output('d')
            self.curr_ki = self.get_param_output('i')
            self.set_sliders()
            self.set_SpinBoxes()
            self.initialize_temps()

    def pitchButtonCallback(self):
        if self.pitchButton.isChecked():
            # Set the tuning mode
            self.tuning_mode = 'p'
            # Get the other parameters from ROS
            self.curr_kp = self.get_param_output('p')
            self.curr_kd = self.get_param_output('d')
            self.curr_ki = self.get_param_output('i')
            self.set_sliders()
            self.set_SpinBoxes()
            self.initialize_temps()

    def airspeedButtonCallback(self):
        if self.airspeedButton.isChecked():
            # Set the tuning mode
            self.tuning_mode = 'a_t'
            # Get the other parameters from ROS
            self.curr_kp = self.get_param_output('p')
            self.curr_kd = self.get_param_output('d')
            self.curr_ki = self.get_param_output('i')
            self.set_sliders()
            self.set_SpinBoxes()
            self.initialize_temps()

    def altitudeButtonCallback(self):
        if self.altitudeButton.isChecked():
            # Set the tuning mode
            self.tuning_mode = 'a'
            # Get the other parameters from ROS
            self.curr_kp = self.get_param_output('p')
            self.curr_kd = self.get_param_output('d')
            self.curr_ki = self.get_param_output('i')
            self.set_sliders()
            self.set_SpinBoxes()
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
        self.kiSlider.setValue(0)
        self.kdSlider.setValue(0)

    def kp_slider_callback(self):
        slider_val = self.kpSlider.value()
        self.temp_kp = self.curr_kp + self.kp_edit_dist * slider_val / 100
        if self.disp: print(self.temp_kp)
        self.kpSpinBox.setValue(self.temp_kp)
    
    def ki_slider_callback(self):
        slider_val = self.kiSlider.value()
        self.temp_ki = self.curr_ki + self.ki_edit_dist * slider_val / 100
        if self.disp: print(self.temp_ki)
        self.kiSpinBox.setValue(self.temp_ki)
   
    def kd_slider_callback(self):
        slider_val = self.kdSlider.value()
        self.temp_kd = self.curr_kd + self.kd_edit_dist * slider_val / 100
        if self.disp: print(slider_val, self.temp_kd)
        self.kdSpinBox.setValue(self.temp_kd)


    def set_SpinBoxes(self):
        # Sliders have an integer range. Set this from +- 100
        self.kpSpinBox.setMinimum(self.curr_kp - self.kp_edit_dist)
        self.kpSpinBox.setMaximum(self.curr_kp + self.kp_edit_dist)
        self.kpSpinBox.setValue(self.curr_kp)

        self.kiSpinBox.setMinimum(self.curr_ki - self.ki_edit_dist)
        self.kiSpinBox.setMaximum(self.curr_ki + self.ki_edit_dist)
        self.kiSpinBox.setValue(self.curr_ki)

        self.kdSpinBox.setMinimum(self.curr_kd - self.kd_edit_dist)
        self.kdSpinBox.setMaximum(self.curr_kd + self.kd_edit_dist)
        self.kdSpinBox.setValue(self.curr_kd)

    def kpSpinBox_callback(self):
        kpSpinBox_value = self.kpSpinBox.value()
        self.temp_kp = kpSpinBox_value
        slider_val = (self.temp_kp - self.curr_kp)*100/self.kp_edit_dist
        self.kpSlider.setValue(int(slider_val))

    def kiSpinBox_callback(self):
        kiSpinBox_value = self.kiSpinBox.value()
        self.temp_ki = kiSpinBox_value
        slider_val = (self.temp_ki - self.curr_ki)*100/self.ki_edit_dist
        self.kiSlider.setValue(int(slider_val))

    def kdSpinBox_callback(self):
        kdSpinBox_value = self.kdSpinBox.value()
        self.temp_kd = kdSpinBox_value
        slider_val = (self.temp_kd - self.curr_kd)*100/self.kd_edit_dist
        self.kdSlider.setValue(int(slider_val))
       

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
        
        # Reinitialize the gui
        self.set_sliders()
        self.set_SpinBoxes()
        self.initialize_temps()


# Main loop
if __name__=='__main__':
    app = QApplication(sys.argv)
    window = Window()
    window.show()
    sys.exit(app.exec_())
