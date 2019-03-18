#!/usr/bin/python

import rospy
import miro2 as miro
import time
import sys
import os
import numpy as np

################################################################

def error(msg):

	print(msg)
	sys.exit(0)

def fmt(msg):

	x = msg.data
	s = "{0:.3f} {1:.3f}".format(x[0], x[1])
	return s

################################################################

class client:

	def callback_sensors(self, msg):

		cliff = (np.array([msg.cliff.data[0], msg.cliff.data[1]]) * 15.0).astype('int')
		print cliff

	def loop(self):

		# loop
		while not rospy.core.is_shutdown():

			# state
			time.sleep(0.1)

	def __init__(self):

		# state
		self.wait = False

		# robot name
		topic_base = "/" + os.getenv("MIRO_ROBOT_NAME") + "/"

		# subscribe
		topic = topic_base + "sensors/package"
		print ("subscribe", topic)
		self.sub_log = rospy.Subscriber(topic, miro.msg.sensors_package, self.callback_sensors)

if __name__ == "__main__":

	rospy.init_node("client_show_cliff", anonymous=True)
	main = client()
	main.loop()




