#!/usr/bin/python

import rospy
import time
import sys
import os
import numpy as np

import std_msgs
import miro2 as miro

################################################################

def error(msg):

	print(msg)
	sys.exit(0)

################################################################

class client_sonar:

	def callback_package(self, msg):

		x = msg.sonar.range

		print "sonar", x

		#time.sleep(1)

	def loop(self):

		# loop
		while not rospy.core.is_shutdown():

			# sleep
			time.sleep(0.01)

	def __init__(self):

		# config

		# robot name
		topic_base = "/" + os.getenv("MIRO_ROBOT_NAME") + "/"

		# subscribe
		topic = topic_base + "sensors/package"
		print ("subscribe", topic)
		self.sub_package = rospy.Subscriber(topic, miro.msg.sensors_package, self.callback_package)

if __name__ == "__main__":

	rospy.init_node("client_sonar", anonymous=True)
	main = client_sonar()
	main.loop()




