import os
import rospy
import rospkg
import csv

from std_msgs.msg import UInt16, UInt8
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore
#from python_qt_binding.QtGui import QGridLayout
from python_qt_binding.QtWidgets import QWidget, QGridLayout, QPushButton

class MonitorPlugin(Plugin):
    sensor0_id = 0
    sensor0_sum = 0
    sensor1_id = 0
    sensor1_sum = 0
    do_update_ui = QtCore.pyqtSignal(int, int, int)
    def __init__(self, context):
        super(MonitorPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MonitorPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_monitor_robochoto'), 'resource', 'MonitorPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MonitorPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        print("subscribiendose a los topicos")
        self._sensor0_sub = rospy.Subscriber("/robot/sensor_0_value", UInt16, self._sensor0_cb)
        self._sensor1_sub = rospy.Subscriber("/robot/sensor_1_value", UInt16, self._sensor1_cb)
        self._detection_sub = rospy.Subscriber("/robot/mine_detected", UInt8, self._detection_cb)
        
        self.do_update_ui.connect(self._update_ui)



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
    
    def _update_ui(self, s0val, s1val, detect):

        if detect == 0:
            self._widget.sensor0_event.setStyleSheet('background: green')
            self._widget.sensor1_event.setStyleSheet('background: green')
        elif detect == 1:
            self._widget.sensor0_event.setStyleSheet('background: red')
            self._widget.sensor1_event.setStyleSheet('background: green')
        elif detect == 2:
            self._widget.sensor0_event.setStyleSheet('background: green')
            self._widget.sensor1_event.setStyleSheet('background: red')
        elif detect == 3:
            self._widget.sensor0_event.setStyleSheet('background: red')
            self._widget.sensor1_event.setStyleSheet('background: red')

        if s0val >=0:
            self._widget.front_sensor_level.setValue(s0val)
            self._widget.superficial_counter.display(s0val)
        if s1val >=0:
            self._widget.rear_sensor_level.setValue(s1val)
            self._widget.burried_counter.display(s1val)
        

    def _button_cb(self):
        print("hola")
        #idbutton = self._widget.sender()
    
    def _detection_cb(self, msg):
        self.do_update_ui.emit(-1, -1, msg.data)

            

    def _sensor0_cb(self, msg):
        self.sensor0_sum += msg.data / 10
        self.sensor0_id += 1

        if self.sensor0_id > 5:
            avg = self.sensor0_sum / self.sensor0_id
            self.do_update_ui.emit(avg, -1, 5)
            #self._widget.front_sensor_level.setText("")
            self.sensor0_id = 0
            self.sensor0_sum = 0

    def _sensor1_cb(self, msg):
        self.sensor1_sum += msg.data / 10
        self.sensor1_id += 1

        if self.sensor1_id > 5:
            avg = self.sensor1_sum / self.sensor1_id
            self.do_update_ui.emit(-1, avg, 5)
            self.sensor1_id = 0
            self.sensor1_sum = 0
        #self._widget.rear_sensor_level.setValue(msg.data * (100.0/1023))
        #self._widget.burried_counter.intValue = msg.data
    