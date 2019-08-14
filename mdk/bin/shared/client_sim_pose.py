#!/usr/bin/python

import rospy
import time
import sys
import os
import numpy as np

import std_msgs
from geometry_msgs.msg import Pose2D

################################################################

def error(msg):

	print(msg)
	sys.exit(0)

################################################################

class client_mics:

	def callback_pose(self, msg):

		mag = msg
		print "received", mag

	def loop(self):

		# loop
		while not rospy.core.is_shutdown():

			# sleep
			time.sleep(0.01)

	def __init__(self):

		# config
		self.mics = []

		# robot name
		topic_base = "/" + os.getenv("MIRO_ROBOT_NAME") + "/"

		# subscribe
		topic = topic_base + "sensors/body_pose"
		print ("subscribe", topic)
		self.sub_mics = rospy.Subscriber(topic, Pose2D, self.callback_pose)

if __name__ == "__main__":

	rospy.init_node("client_mics", anonymous=True)
	main = client_mics()
	main.loop()




