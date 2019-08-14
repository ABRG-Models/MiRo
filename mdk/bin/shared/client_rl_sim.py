#!/usr/bin/python

import rospy
import time
import sys
import os
import numpy as np

import std_msgs
import miro2 as miro
from geometry_msgs.msg import Pose2D

################################################################

def error(msg):

	print(msg)
	sys.exit(0)

################################################################

class client_sonar:

	def callback_package(self, msg):

		x = msg.sonar.range

		print "sonar", x


	def callback_pose(self, msg):

		self.pose = msg
		print "received pose", self.pose

	def loop(self):

		# loop
		while not rospy.core.is_shutdown():

			# sleep
			time.sleep(0.01)

	def __init__(self):

		# config
		self.pose = []
		# robot name
		topic_base = "/" + os.getenv("MIRO_ROBOT_NAME") + "/"

		# subscribe
		self.sub_package = rospy.Subscriber(topic_base + "sensors/package", miro.msg.sensors_package, self.callback_package)
		print ("subscribe", topic_base + "sensors/package")
		self.sub_pose = rospy.Subscriber(topic_base + "sensors/body_pose", Pose2D, self.callback_pose)
		print ("subscribe", topic_base + "sensors/body_pose")



if __name__ == "__main__":

	rospy.init_node("client_sonar", anonymous=True)
	main = client_sonar()
	main.loop()




