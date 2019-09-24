import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from QtCore import QThread
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QMessageBox, QDialog
from python_qt_binding.QtCore import Qt, QTimer, Slot, QBasicTimer, Signal
from python_qt_binding.QtGui import QPixmap

import time

import argparse
import tempfile
from multiprocessing import Queue as queue
import sys
from gtts import gTTS
from playsound import playsound
import speech_recognition as sr
import time
from os import path
import keyboard

import sounddevice as sd
import soundfile as sf
import numpy
assert numpy

# https://medium.com/@webmamoffice/getting-started-gui-s-with-python-pyqt-qthread-class-1b796203c18c
class SpeechRecognitionThread(QThread):
    # rec_done_sig = Signal([bool, str])
    rec_done_sig = Signal([str])
    def __init__(self, parent=None):
        QThread.__init__(self, parent)
        self._is_rec_btn_pressed = False
        self.q = queue()
        self.robo = {}

    def on_rec_btn_press(self, state):
        self._is_rec_btn_pressed = state

    def callback(self, indata, frames, time, status):
        # print(status, file=sys.stderr)
        # if status:
            # print(status, file=sys.stderr)
        self.q.put(indata.copy())

    def run(self):
        self.running = True
        # while self.running:
        #    time.sleep(1)
        device_info = sd.query_devices(None, 'input')
        filename = tempfile.mktemp(prefix='Human_',suffix='.wav', dir='/home/robocup2019/movr/movr_ws/src/movr_ocs/src/rqt_movr_speech_rec')
        samplerate = int(device_info['default_samplerate'])

        with sf.SoundFile(filename, mode='x', samplerate=samplerate,
                    channels=1, subtype="PCM_24") as file:
            with sd.InputStream(samplerate=samplerate, device=None,
                                channels=1, callback=self.callback):       
                while self.running:
                    file.write(self.q.get())            
        # self.rec_done_sig.emit("DONE RECORDING...")
        self.rec_done_sig.emit("DONE RECORDING...")
        print('\nRecording finished: ' + repr(filename))
        AUDIO_FILE = path.join(path.dirname(path.realpath(__file__)), filename)
        r = sr.Recognizer()
        with sr.AudioFile(AUDIO_FILE) as source:
            audio = r.record(source)
        text = r.recognize_google(audio)
        print(text)
        tts = gTTS(text, 'en')
        text = text.replace(" ", "_")
        robotFile = text + '.mp3'
        if not os.path.exists('/home/robocup2019/movr/movr_ws/src/movr_ocs/src/rqt_movr_speech_rec' + robotFile):
            print('Saving new phrase...')
            tts.save('/home/robocup2019/movr/movr_ws/src/movr_ocs/src/rqt_movr_speech_rec' + robotFile)
            if robotFile not in self.robo:
                self.robo[robotFile] = robotFile
        playsound('/home/robocup2019/movr/movr_ws/src/movr_ocs/src/rqt_movr_speech_rec' + robotFile)
        robotFile = ''
        self.rec_done_sig.emit("DONE RECORDING...")

class MOVRSpeechRecPlugin(Plugin):

    btn_rec_state_changed_sig = Signal([bool])

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
        # self.btn_rec_state_changed_sig.connect(self.on_btn_state_changed)
        self._widget.btn_record.clicked.connect(self.on_btn_rec_clicked)

        self._white_string  = "background-color: rgb(255,255,255); color: #000;"
        self._red_string    = "background-color: rgb(255,0,0); color: #FFF;"
        self._orange_string = "background-color: rgb(255,128,0)"
        self._yellow_string = "background-color: rgb(255,255,0)"
        self._green_string  = "background-color: rgb(128,255,0)"
        self._black_string  = "background-color: rgb(0,0,0)"
        self._grey_string   = "background-color: rgb(160,160,160); color: #606060"

    def on_btn_rec_clicked(self):

        if not self.btn_clicked:
            self.btn_clicked = True
            # Launch SpeechRecognition Thread
            self.rec_thread = SpeechRecognitionThread()
            # Connect (this) signal to a function in the thread
            self.btn_rec_state_changed_sig.connect(self.rec_thread.on_rec_btn_press)
            # Tell the thread that we have clicked something (set the state)
            self.btn_rec_state_changed_sig.emit(self.btn_clicked)
            # Start thread
            self.rec_thread.start()

            # Connect thread's signal to a function in (this) class
            self.rec_thread.rec_done_sig.connect(self.update_rec_info)
            # Change button's color
            self._widget.btn_record.setStyleSheet(self._green_string)
            self._widget.text_logs.setPlainText("RECORDING")

        else:
            self.rec_thread.running = False
            # self.but1.setEnabled(True)           
            self._widget.btn_record.setStyleSheet(self._red_string)
            self.btn_clicked = False     
            self._widget.text_logs.setPlainText("STOPPING RECORDING...")

            # self.btn_rec_state_changed.emit(self.btn_clicked)
            # self.btn_clicked = not self.btn_clicked

    def update_rec_info(self, str):
        '''
        Function used by thread when it is done runnning
        '''
        self._widget.text_logs.setPlainText(str)
        rospy.loginfo(str)
        rospy.loginfo("DONE!!!")

    # def update_button_state(self, state):
    #     if state == True:
    #         self._widget.btn_record.setStyleSheet(self._green_string)
    #     else:
    #         self._widget.btn_record.setStyleSheet(self._red_string)        

    # def on_btn_state_changed(self, state):
    #     if state == True:
    #         self._widget.btn_record.setStyleSheet(self._green_string)
    #     else:
    #         self._widget.btn_record.setStyleSheet(self._red_string)