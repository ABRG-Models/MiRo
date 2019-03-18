#!/usr/bin/python

import rospy
import miro2 as miro
import time
import sys
import os
import geometry_msgs

################################################################

def error(msg):

	print(msg)
	sys.exit(0)

def fmt(x):

	s = "{0:.3f} {1:.3f}".format(x[0], x[1])
	return s

################################################################

class client:

	def callback_cmd_vel(self, msg):

		x = [msg.twist.linear.x, msg.twist.angular.z]

		if x[0] != 0.0 or x[1] != 0.0:
			self.wait = False
			print fmt(x)

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
		topic = topic_base + "control/cmd_vel"
		print ("subscribe", topic)
		self.sub_cmd_vel = rospy.Subscriber(topic, geometry_msgs.msg.TwistStamped, self.callback_cmd_vel)

if __name__ == "__main__":

	rospy.init_node("client_show_cmd_vel", anonymous=True)
	main = client()
	main.loop()




