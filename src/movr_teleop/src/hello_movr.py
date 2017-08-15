#!/usr/bin/env python
import os
import ConfigParser
import io
import string
import hashlib
import sys

import time
import json
import pygame
import apiai
import speech_recognition as sr 
from gtts import gTTS

import rospy
from std_msgs.msg import String 
from ackermann_msgs.msg import AckermannDrive


'''
moveBindings is 
a mapping between what the robot understood and a tuple command (steering_angle, speed)
i.e.
	'Ending our journey': (0.0, 130.0)

Currently speed is hardcoded to represent the analog value that DA converter in Arduino receives:


-45 <= steering_angle <= 45

REVERSE
speed = 210 			4.00 Volts
speed = 127 			2.49 Volts			KICKSTART REVERSE
127 <= speed <= 132   	2.4 - 2.5 Volts
speed = 160 			3.13 Volts 			MID-HIGH SPEED
speed = 180 			3.53 Volts			HIGH SPEED

FORWARD
speed = 96 				1.875 Volts			KICKSTART FORWARD
speed = 60				1.168 Volts			MID-HIGH SPEED
speed = 50 				0.968 Volts     	HIGH SPEED

STOP
speed = 125 - 126 		2.45 - 2.46 Volts	FULL STOP

For the speed value, anything below 2.4 Volts will stop the vehicle and above 1.9

'''
moveBindings = {
	'Ending our journey': 	(0.0, 120.0),
	'Stopping': 			(0.0, 120.0),
	'Ok moving forward': 	(0.0, 80.0),
	'Going forward': 		(0.0, 80.0),
	'Sure thing. Moving.': 	(0.0, 80.0)
}

rospy.init_node('speech_command_pub')
pub = rospy.Publisher('movr_cmd', AckermannDrive, queue_size=1)

def speech_recog_cb(recognizer, audio):
	print("*Robot listening...*")
	try:
		recgonized_speech = recognizer.recognize_google(audio)
		print("*Robot thinks you said: " + recgonized_speech + "*")

		request = ai.text_request()
		request.query = recgonized_speech
		response = request.getresponse()
		response_str = response.read()

		response_json = json.loads(response_str)
		robot_response = response_json['result']['fulfillment']['speech']
		print(robot_response)

		# checksum = sha256_checksum(robot_response)
		file_name = robot_response.translate({ord(i):None for i in ' .,;@#$!?\"\''}) + '.mp3'

		if robot_response in moveBindings.keys():
			steering_angle = moveBindings[robot_response][0]
			speed = moveBindings[robot_response][1]

			ackermann_twist = AckermannDrive()
			ackermann_twist.steering_angle = steering_angle
			ackermann_twist.speed = speed

			pub.publish(ackermann_twist)


		if not os.path.isfile('./' + file_name):
			print("Saving new response...")
			tts = gTTS(robot_response)  
			tts.save(file_name)

		pygame.mixer.init()
		pygame.mixer.music.load(file_name)
		print("*Robot speaks.*")
		pygame.mixer.music.play()

		print("*Robot speaking...*")
		time.sleep(4)
		print("*Robot done speaking...*")

	except sr.UnknownValueError:
		print("Speech Recognition could not understand audio")
	except sr.RequestError as e:
		print("Could not request results from Speech Recognition service; {0}".format(e))

def sha256_checksum(text):
	sha256 = hashlib.sha256()
	sha256.update(text)
	return sha256.hexdigest()

if __name__ == '__main__':

	# with open("./movr.ini") as f:
	# 	movr_config = f.read()
	# config = ConfigParser.RawConfigParser(allow_no_value=True)
	# config.readfp(io.BytesIO(movr_config))

	# ai 		= apiai.ApiAI(config.get('api_ai', 'client_access_token'))
	# request = ai.text_request()
	# request.session_id = config.get('api_ai', 'session_id')

	ai 		= apiai.ApiAI('17dac27f07a24b47b39aece331efc3ad')
	request = ai.text_request()
	request.session_id = 'api_ai', '<SESSION ID, UNIQUE FOR EACH USER>'

	# main()

	r = sr.Recognizer()
	m = sr.Microphone()
	
	with m as source:
		r.adjust_for_ambient_noise(source)

	# r.pause_threshold = 1
	r.operation_timeout = 8
	stop_listening = r.listen_in_background(m, speech_recog_cb)

	# do some other computation for 5 seconds, then stop listening and keep doing other computations
	for _ in range(50): time.sleep(0.1) # we're still listening even though the main thread is doing other things
	# stop_listening()  # calling this function requests that the background listener stop listening
	while True: time.sleep(0.1)

	print("Initiating voice agent...")