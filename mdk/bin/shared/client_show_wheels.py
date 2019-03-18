#!/usr/bin/python

import rospy
import miro2 as miro
import time
import sys
import os

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

		x = msg.wheel_speed_cmd.data
		disp = x[0] != 0.0 or x[1] != 0.0

		if disp:
			self.wait = False
			print \
				fmt(msg.wheel_speed_cmd), \
				fmt(msg.wheel_speed_opto), \
				fmt(msg.wheel_speed_back_emf), \
				fmt(msg.wheel_effort_pwm)
		else:
			if not self.wait:
				print "________________________________________________________________\n"
			self.wait = True

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

	rospy.init_node("client_show_wheels", anonymous=True)
	main = client()
	main.loop()




