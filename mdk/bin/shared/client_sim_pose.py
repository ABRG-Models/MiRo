#!/usr/bin/python

import rospy
import time
import sys
import os
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

		start = time.clock()

		# loop
		while not rospy.core.is_shutdown():
			print "received", self.pose
			end = time.clock()
			self.velocity.twist.linear.x=0.1
			self.pub_cmd_vel.publish(self.velocity)

			if (end-start)%5==0:
				self.velocity.twist.linear.x=0.0
				self.pub_cmd_vel.publish(self.velocity)
			# sleep
			time.sleep(1)
			start+=1
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




