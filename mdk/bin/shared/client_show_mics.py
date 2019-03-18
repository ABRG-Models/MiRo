#!/usr/bin/python

import rospy
import time
import sys
import os
import numpy as np

import std_msgs

################################################################

def error(msg):

	print(msg)
	sys.exit(0)

################################################################

class client_mics:

	def callback_mics(self, msg):

		mag = (np.mean(np.abs(np.array(msg.data))) / 32768.0 * 100.0).astype('int')
		print "received", len(msg.data), "samples with mean mag", mag, "%"

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
		topic = topic_base + "sensors/mics"
		print ("subscribe", topic)
		self.sub_mics = rospy.Subscriber(topic, std_msgs.msg.Int16MultiArray, self.callback_mics)

if __name__ == "__main__":

	rospy.init_node("client_mics", anonymous=True)
	main = client_mics()
	main.loop()




