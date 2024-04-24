import os
import rclpy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMainWindow, QWidget, QSizePolicy, QHBoxLayout, QVBoxLayout, QRadioButton, QPushButton, QLabel, QLayout, QSlider, QDoubleSpinBox
from python_qt_binding.QtCore import QSize, Qt, QRect

from ament_index_python import get_resource

class ROSflightGUI(Plugin):
    def __init__(self, context):
        super(ROSflightGUI, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ROSflightGUI')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns) 

        # Create QWidget
        self._widget = QMainWindow()
        # Get path to UI file which should be in the "resource" folder of this package
        _, path = get_resource('packages', 'rosplane_tuning')
        ui_file = os.path.join(path, 'share', 'rosplane_tuning', 'resource', 'tuning_gui.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('ROSflightTuningUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # self.setupUi()

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
    
    def setupUi(self):
        self.centralwidget = QWidget(self._widget)
        sizePolicy = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.centralwidget.sizePolicy().hasHeightForWidth())
        self.centralwidget.setSizePolicy(sizePolicy)
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayoutWidget = QWidget(self.centralwidget)
        self.horizontalLayoutWidget.setGeometry(QRect(20, 10, 761, 571))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout = QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setSizeConstraint(QLayout.SetFixedSize)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setSpacing(6)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setSizeConstraint(QLayout.SetMaximumSize)
        self.verticalLayout.setContentsMargins(-1, -1, 50, -1)
        self.verticalLayout.setSpacing(6)
        self.verticalLayout.setObjectName("verticalLayout")
        self.CourseButton = QRadioButton(self.horizontalLayoutWidget)
        self.CourseButton.setObjectName("CourseButton")
        self.verticalLayout.addWidget(self.CourseButton)
        self.rollButton = QRadioButton(self.horizontalLayoutWidget)
        self.rollButton.setObjectName("rollButton")
        self.verticalLayout.addWidget(self.rollButton)
        self.pitchButton = QRadioButton(self.horizontalLayoutWidget)
        self.pitchButton.setObjectName("pitchButton")
        self.verticalLayout.addWidget(self.pitchButton)
        self.airspeedButton = QRadioButton(self.horizontalLayoutWidget)
        self.airspeedButton.setObjectName("airspeedButton")
        self.verticalLayout.addWidget(self.airspeedButton)
        self.altitudeButton = QRadioButton(self.horizontalLayoutWidget)
        self.altitudeButton.setObjectName("altitudeButton")
        self.verticalLayout.addWidget(self.altitudeButton)
        self.runButton = QPushButton(self.horizontalLayoutWidget)
        self.runButton.setObjectName("runButton")
        self.verticalLayout.addWidget(self.runButton)
        self.saveButton = QPushButton(self.horizontalLayoutWidget)
        self.saveButton.setObjectName("saveButton")
        self.verticalLayout.addWidget(self.saveButton)
        self.clearButton = QPushButton(self.horizontalLayoutWidget)
        self.clearButton.setObjectName("Clear")
        self.verticalLayout.addWidget(self.clearButton)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.verticalLayout_4 = QVBoxLayout()
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.label_2 = QLabel(self.horizontalLayoutWidget)
        self.label_2.setObjectName("label_2")
        self.verticalLayout_4.addWidget(self.label_2)
        self.label = QLabel(self.horizontalLayoutWidget)
        self.label.setObjectName("label")
        self.verticalLayout_4.addWidget(self.label)
        self.label_3 = QLabel(self.horizontalLayoutWidget)
        self.label_3.setObjectName("label_3")
        self.verticalLayout_4.addWidget(self.label_3)
        self.horizontalLayout_3.addLayout(self.verticalLayout_4)
        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setSizeConstraint(QLayout.SetDefaultConstraint)
        self.verticalLayout_3.setContentsMargins(0, -1, 0, -1)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.kpSlider = QSlider(self.horizontalLayoutWidget)
        sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.kpSlider.sizePolicy().hasHeightForWidth())
        self.kpSlider.setSizePolicy(sizePolicy)
        self.kpSlider.setMaximumSize(QSize(400, 50))
        self.kpSlider.setOrientation(Qt.Horizontal)
        self.kpSlider.setObjectName("kpSlider")
        self.verticalLayout_3.addWidget(self.kpSlider)
        self.kiSlider = QSlider(self.horizontalLayoutWidget)
        sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.kiSlider.sizePolicy().hasHeightForWidth())
        self.kiSlider.setSizePolicy(sizePolicy)
        self.kiSlider.setMaximumSize(QSize(400, 50))
        self.kiSlider.setOrientation(Qt.Horizontal)
        self.kiSlider.setObjectName("kiSlider")
        self.verticalLayout_3.addWidget(self.kiSlider)
        self.kdSlider = QSlider(self.horizontalLayoutWidget)
        sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.kdSlider.sizePolicy().hasHeightForWidth())
        self.kdSlider.setSizePolicy(sizePolicy)
        self.kdSlider.setMaximumSize(QSize(400, 50))
        self.kdSlider.setOrientation(Qt.Horizontal)
        self.kdSlider.setObjectName("kdSlider")
        self.verticalLayout_3.addWidget(self.kdSlider)
        self.horizontalLayout_3.addLayout(self.verticalLayout_3)
        self.horizontalLayout.addLayout(self.horizontalLayout_3)
        self.verticalLayout_5 = QVBoxLayout()
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.kpSpinBox = QDoubleSpinBox(self.horizontalLayoutWidget)
        self.kpSpinBox.setObjectName("kpSpinBox")
        self.verticalLayout_5.addWidget(self.kpSpinBox)
        self.kiSpinBox = QDoubleSpinBox(self.horizontalLayoutWidget)
        self.kiSpinBox.setObjectName("kiSpinBox")
        self.verticalLayout_5.addWidget(self.kiSpinBox)
        self.kdSpinBox = QDoubleSpinBox(self.horizontalLayoutWidget)
        self.kdSpinBox.setObjectName("kdSpinBox")
        self.verticalLayout_5.addWidget(self.kdSpinBox)
        self.horizontalLayout.addLayout(self.verticalLayout_5)