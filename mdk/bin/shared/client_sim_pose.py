#!/usr/bin/python

import rospy
import time
import sys
import os#
import datetime
import numpy as np

import std_msgs
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import TwistStamped

################################################################

def error(msg):

	print(msg)
	sys.exit(0)

################################################################

class client_mics:

	def callback_pose(self, msg):

		self.pose = msg
		# print "received", mag

	def loop(self):

		start = datetime.datetime.now()
		restart = False

		# loop
		while not rospy.core.is_shutdown():
			print "received", self.pose
			end = datetime.datetime.now()

			if (end-start).seconds > 5:
				self.velocity.twist.linear.x=0.0
				self.pub_cmd_vel.publish(self.velocity)
				restart = True
				time.sleep(1)
			else:
				self.velocity.twist.linear.x = 0.1
				self.pub_cmd_vel.publish(self.velocity)
				restart = False

			if (end-start).seconds>6 and restart == True:
				start = datetime.datetime.now()

		# sleep
			#time.sleep(1)
			# self.velocity.twist.linear.x=0.0
			# self.pub_cmd_vel.publish(self.velocity)



	def __init__(self):

		# config
		self.pose = []

		# robot name
		topic_base = "/" + os.getenv("MIRO_ROBOT_NAME") + "/"

		self.velocity = TwistStamped()

		self.pub_cmd_vel = rospy.Publisher(topic_base + "control/cmd_vel", TwistStamped, queue_size=0)

		# subscribe
		topic = topic_base + "sensors/body_pose"
		print ("subscribe", topic)
		self.sub_mics = rospy.Subscriber(topic, Pose2D, self.callback_pose)

if __name__ == "__main__":

	rospy.init_node("client_mics", anonymous=True)
	main = client_mics()
	main.loop()



