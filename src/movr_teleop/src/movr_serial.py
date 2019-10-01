#!/usr/bin/env python

import serial
import time
import sys

class MOVRSerial:
    class __MOVRSerial:
        def __init__(self, port, baudrate):
            self.port = port
            self.baudrate = baudrate
            self.ser = serial.Serial(port, baudrate, timeout=.1)
        def __str__(self):
            return repr(self) + str(self.port) + ' ' + str(self.baudrate) + ' '
    
        def send_drive_command(self, steering, acceleration):
            cmd = "<c " + str(steering) + " " + str(acceleration)  + ">"
            self.ser.write(cmd.encode('utf-8'))

        def send_voice_commmand(self, voice_text):
            cmd = "<v " + voice_text
            self.ser.write(cmd.encode('utf-8'))   

    instance = None

    def __init__(self, port='/dev/ttyUSB0', baudrate=9600):
        """
        rosserial default baudrate is 9600
        edit rosserial library to increase baudrate
        """
        if not MOVRSerial.instance:
            MOVRSerial.instance = MOVRSerial.__MOVRSerial(port, baudrate)
        else:
            MOVRSerial.instance.port = port
            MOVRSerial.instance.baudrate = baudrate

    def __getattr__(self, name):
        return getattr(self.instance, name)