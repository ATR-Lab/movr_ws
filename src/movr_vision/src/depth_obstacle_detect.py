#!/usr/bin/env python

'''
$ roslaunch openni_launch openni.launch
$ rosrun movr_vision depth_obstacle_detect.py
'''
from __future__ import print_function
import sys
import roslib
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def image_cb(data):
	try:
		cv_image = bridge.imgmsg_to_cv2(data, '32FC1')
	except CvBridgeError as e:
		print(e)

	# (rows, cols, channels) = cv_image.shape
	(rows, cols) = cv_image.shape
	if cols > 60 and rows > 60:
		cv2.circle(cv_image, (50, 50), 10, 255)

	cv2.imshow('Image Window', cv_image)
	cv2.waitKey(3)

	try:
		pub.publish(bridge.cv2_to_imgmsg(cv_image, '32FC1'))
	except CvBridgeError as e:
		print(e)

if __name__ == '__main__':
	rospy.init_node('image_converter', anonymous=True)
	pub = rospy.Publisher('image_topic_2', Image)
	bridge = CvBridge()
	# sub = rospy.Subscriber('/camera/rgb/image_raw', Image, image_cb) #///camera/rgb/image_color
	sub = rospy.Subscriber('/camera/depth/image', Image, image_cb)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print('Shutting down')