import os
import rospy
import rospkg
import csv


from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore
#from python_qt_binding.QtGui import QGridLayout
from python_qt_binding.QtWidgets import QWidget, QGridLayout, QPushButton

class MineMapPlugin(Plugin):
    eventCase = 0
    nrows = 20
    ncols = 20

    map_values = [[0 for c in range(ncols)] for r in range(nrows)]
    print("dimension del mapa: {}x{}".format(len(map_values), len(map_values[0])))
    rows = ['20','19','18','17','16','15','14','13','12','11','10','09','08','07','06','05','04','03','02','01']
    cols = 'TSRQPONMLKJIHGFEDCBA'
    val_labels = ['vacio', 'superficial', 'enterrada']
    def __init__(self, context):
        super(MineMapPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MineMapPlugin')

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
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_robochoto'), 'resource', 'MineMapPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MineMapPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        #self._widget.test.setCheckable(True)
        self._widget.resetbtn.clicked.connect(self._on_click)
        self._widget.reportbtn.clicked.connect(self._make_report)
        
        self.mapa_ui = self._widget.findChild(QGridLayout, "mapa")
        boton = self._widget.findChild(QPushButton, "b")
        boton.clicked.connect(self._on_click)
        for i in range(2, 401):
            butt = self._widget.findChild(QPushButton, "b_"+str(i))
            butt.clicked.connect(self._on_click)

        print(boton.text())
        print(self.mapa_ui)
        self.map_dict = {}


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
    def _button_cb(self):
        print("hola")
        #idbutton = self._widget.sender()
        
        if self.eventCase:
            self._widget.test.setText("hola")
            self._widget.test.setStyleSheet("background-color: red")
            self.eventCase = False
        else:
            self._widget.test.setText("nada")
            self._widget.test.setStyleSheet("background-color: green")
            self.eventCase = True

    def _make_report(self):
        print("REPORTE DE MINAS:")
        for key in self.map_dict:
            print(key, self.map_dict[key])
        
        with open('mineReport.csv', 'w') as f:
            for key in self.map_dict.keys():
                f.write("%s,%s\n"%(key,self.map_dict[key]))
        
        
    @QtCore.pyqtSlot()
    def _on_click(self):
        
        aux_but = self.sender()
        id = self.mapa_ui.indexOf(aux_but)
        r, c = self.mapa_ui.getItemPosition(id)[:2]
        val = self.map_values[r][c]

        key = self.cols[c] + self.rows[r]

        if val == 0:
            self.sender().setStyleSheet("background-color: yellow")
            self.sender().setText("S")
            self.map_values[r][c] = 1
            self.map_dict[key] = self.val_labels[1]
            print(key, self.map_dict[key])

        elif val == 1:
            self.sender().setStyleSheet("background-color: red")
            self.sender().setText("E")
            self.map_values[r][c] = 2
            self.map_dict[key] = self.val_labels[2]
            print(key, self.map_dict[key])
        elif val == 2:
            self.sender().setStyleSheet("background-color: green")
            self.sender().setText("V")
            self.map_values[r][c] = 0
            self.map_dict[key] = self.val_labels[0]
            print(key, self.map_dict[key])