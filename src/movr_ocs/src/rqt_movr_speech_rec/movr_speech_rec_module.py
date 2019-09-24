import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QMessageBox, QDialog
from python_qt_binding.QtCore import Qt, QTimer, Slot, QBasicTimer, Signal
from python_qt_binding.QtGui import QPixmap

class MOVRSpeechRecPlugin(Plugin):

    btn_rec_state_changed = Signal([bool])

    def __init__(self, context):
        super(MOVRSpeechRecPlugin, self).__init__(context)
        self.setObjectName('MOVRSpeechRecPlugin')

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
        ui_file = os.path.join(rospkg.RosPack().get_path('movr_ocs'), 'resource', 'MOVRSpeechRecPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MOVRSpeechRecPlugin')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)


        self.btn_clicked = False
        # Add Event Handler
        self.btn_rec_state_changed.connect(self.on_btn_state_changed)
        self._widget.btn_record.clicked.connect(self.on_btn_rec_clicked)

        self._white_string  = "background-color: rgb(255,255,255); color: #000;"
        self._red_string    = "background-color: rgb(255,0,0); color: #FFF;"
        self._orange_string = "background-color: rgb(255,128,0)"
        self._yellow_string = "background-color: rgb(255,255,0)"
        self._green_string  = "background-color: rgb(128,255,0)"
        self._black_string  = "background-color: rgb(0,0,0)"
        self._grey_string   = "background-color: rgb(160,160,160); color: #606060"

    def on_btn_rec_clicked(self):
        self.btn_rec_state_changed.emit(self.btn_clicked)
        self.btn_clicked = not self.btn_clicked

    def on_btn_state_changed(self, state):
        if state == True:
            self._widget.btn_record.setStyleSheet(self._green_string)
        else:
            self._widget.btn_record.setStyleSheet(self._red_string)