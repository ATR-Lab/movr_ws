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
from os import path
import keyboard

import sounddevice as sd
import soundfile as sf
import numpy
assert numpy

# https://medium.com/@webmamoffice/getting-started-gui-s-with-python-pyqt-qthread-class-1b796203c18c
class SpeechRecognitionThread(QThread):
    rec_done_sig = Signal([str])
    sr_done_sig = Signal([str])
    def __init__(self, parent=None):
        QThread.__init__(self, parent)
        self._is_rec_btn_pressed = False
        self.q = queue()
        self.robo = {}

    def on_rec_btn_press(self, state):
        self._is_rec_btn_pressed = state

    def callback(self, indata, frames, time, status):
        self.q.put(indata.copy())

    def run(self):
        self.running = True
        path = "/home/robocup2019/movr/movr_ws/src/movr_ocs/src/rqt_movr_speech_rec/"
        device_info = sd.query_devices(None, 'input')
        filename = tempfile.mktemp(prefix='Human_', suffix='.wav', dir= path + 'human_files')
        samplerate = int(device_info['default_samplerate'])

        with sf.SoundFile(filename, mode='x', samplerate=samplerate,channels=1, subtype="PCM_24") as file:
            with sd.InputStream(samplerate=samplerate, device=None,channels=1, callback=self.callback):       
                while self.running:
                    file.write(self.q.get())            
        self.rec_done_sig.emit("DONE RECORDING...")
        
        self.rec_done_sig.emit("Recording finished: " + repr(filename))
        r = sr.Recognizer()
        with sr.AudioFile(filename) as source:
            audio = r.record(source)
        transcribed_text = r.recognize_google(audio)
        self.sr_done_sig.emit(transcribed_text)
        tts = gTTS(transcribed_text, 'en')
        transcribed_text = transcribed_text.replace(" ", "_")
        robotFile = transcribed_text + '.mp3'
        if not os.path.exists(path + 'robot_files/' + robotFile):
            self.rec_done_sig.emit("SAVING NEW RECORDING...")
            tts.save(path + 'robot_files/' + robotFile)
            if robotFile not in self.robo:
                self.robo[robotFile] = robotFile
        self.rec_done_sig.emit("PLAYING TTS FILE...")
        playsound(path + 'robot_files/' + robotFile)
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
        parser.add_argument("-q", "--quiet", action="store_true",dest="quiet",help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        self._widget = QWidget() # Create QWidget
        ui_file = os.path.join(rospkg.RosPack().get_path('movr_ocs'), 'resource', 'MOVRSpeechRecPlugin.ui')
        # Get path to UI file which should be in the "resource" folder of this package

        loadUi(ui_file, self._widget)
        # Extend the widget with all attributes and children from UI file

        self._widget.setObjectName('MOVRSpeechRecPlugin')
        # Give QObjects reasonable names

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        context.add_widget(self._widget)
        # Add widget to the user interface

        self.btn_clicked = False
        
        # self.btn_rec_state_changed_sig.connect(self.on_btn_state_changed)

        self._widget.btn_clear.clicked.connect(self.clear_cache)

        self._widget.btn_record.clicked.connect(self.on_btn_rec_clicked)
        # Add Event Handler

        self._red_string    = "background-color: rgb(255,0,0); color: #FFF;"
        self._green_string  = "background-color: rgb(128,255,0)"

    def on_btn_rec_clicked(self):

        if not self.btn_clicked:
            self.btn_clicked = True
            self.rec_thread = SpeechRecognitionThread() # Launch SpeechRecognition Thread
            self.btn_rec_state_changed_sig.connect(self.rec_thread.on_rec_btn_press) 
            # Connect (this) signal to a function in the thread

            self.btn_rec_state_changed_sig.emit(self.btn_clicked)
            # Tell the thread that we have clicked something (set the state)
 
            self.rec_thread.start() # Start thread

            self.rec_thread.rec_done_sig.connect(self.update_rec_info) 
            # Connect thread's signal to a function in (this) class

            self._widget.btn_record.setStyleSheet(self._green_string) # Change button's color
            self._widget.text_logs.setPlainText("RECORDING")

        else:
            self.rec_thread.running = False

            # self.but1.setEnabled(True)    
       
            self._widget.btn_record.setStyleSheet(self._red_string)
            self.btn_clicked = False     
            self._widget.text_logs.setPlainText("STOPPING RECORDING...")
            self.rec_thread.sr_done_sig.connect(self.update_sr_output)

            # self.btn_rec_state_changed.emit(self.btn_clicked)
            # self.btn_clicked = not self.btn_clicked

    def update_rec_info(self, str): # Function used by thread when it is done runnning
        self._widget.text_logs.setPlainText(str)
        rospy.loginfo(str)
        rospy.loginfo("DONE!!!")
    
    def update_sr_output(self, str): # Function used by thread to update output display
        self._widget.text_transcribed_text.setPlainText(str)
        rospy.loginfo(str)
        rospy.loginfo("Updated SR Output.")

    def clear_cache(self): # Function used to clear robot voice cache
        self._widget.text_logs.setPlainText("Clearing Robot and Human Cache")
        os.system("rm -rf /home/robocup2019/movr/movr_ws/src/movr_ocs/src/rqt_movr_speech_rec/robot_files/*.mp3")
        os.system("rm -rf /home/robocup2019/movr/movr_ws/src/movr_ocs/src/rqt_movr_speech_rec/human_files/*.wav")
        rospy.loginfo("Cleared Robot and Human Cache.")

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