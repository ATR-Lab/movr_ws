#!/usr/bin/env python
import os
import sys
import rospy
import rospkg
from std_msgs.msg import String
from gtts import gTTS
from playsound import playsound
from tinydb import TinyDB, Query

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
# get the file path for movr_ocs
movr_ocs_pkg = rospack.get_path('movr_core')

robo = {}

db = TinyDB(movr_ocs_pkg + '/db/db.json')


def op_msg_cb(msg):
    text = msg.data         # Get text from ros message
    tts = gTTS(text, 'en')  # Process text to speech

    text = text.replace(" ", "_")   # Replace space with '-'
    robotFile = text + '.mp3'       # The sound file name

    robot_files_path = movr_ocs_pkg + '/recordings/robot_files/'

    if not os.path.exists(robot_files_path):
        rospy.loginfo('Robot speech recording directory does not exist. Creating directory.')
        os.makedirs(robot_files_path)

    if not os.path.exists(robot_files_path + robotFile):
        tts.save(robot_files_path + robotFile)
        if robotFile not in robo:
            robo[robotFile] = robotFile

    playsound(robot_files_path + robotFile) # Play sound

if __name__ == "__main__":
	
	# args = rospy.myargv(argv=sys.argv)
	# if len(args) != 2:
	# 	port = '/dev/ttyUSB0'
	# else:
	# 	port = args[1]

	rospy.init_node('movr_core_speak')
	sub = rospy.Subscriber("movr_op_msg", String, op_msg_cb)
	rospy.spin()