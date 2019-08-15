#!/usr/bin/python
#
#	{{##file_header##}}
#
#	This is a MIRO ROS client for herding Scottish children.
#
#	This example implements a Braitenberg vehicle type 2a.
#	https://en.wikipedia.org/wiki/Braitenberg_vehicle

import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import CompressedImage

import math
import numpy as np
import time
import sys
import os

from ball_detector import *

import cv2
from cv_bridge import CvBridge, CvBridgeError

import miro2 as miro

################################################################

def error(msg):

	print(msg)
	sys.exit(0)

################################################################

class controller:

	def callback_package(self, msg):

		# ignore until active
		if not self.active:
			return

		# store
		self.package = msg

	def callback_cam(self, ros_image, index):

		# silently (ish) handle corrupted JPEG frames
		try:

			# convert compressed ROS image to raw CV image
			image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "rgb8")

			# do zoom
			#image = cv2.resize(image, (int(image.shape[1] * 0.5), int(image.shape[0] * 0.5)))

			# store image
			self.im[index] = image

		except CvBridgeError as e:

			# swallow error, silently
			#print(e)
			pass

	def callback_caml(self, ros_image):

		self.callback_cam(ros_image, 0)

	def callback_camr(self, ros_image):

		self.callback_cam(ros_image, 1)

	def loop(self):

		# output
		msg_cmd_vel = TwistStamped()

		# desired wheel speed (m/sec)
		wheel_speed = [0.0, 0.0]

		# loop
		while self.active and not rospy.core.is_shutdown():

			"""

			# if sensor data has arrived
			if not self.package is None:

				# get light levels from sensor package (normalised in [0, 1])
				light = self.package.light.data

				# clear sensor package that has now been used
				self.package = None

				# convert to wheel speed (m/sec)
				k = 0.4 
				l = light[0] #specify light sensor
				wheel_speed[0] = k * l 
				wheel_speed[1] = -k * l

				# convert wheel speed to command velocity (m/sec, Rad/sec)
				(dr, dtheta) = miro.utils.wheel_speed2cmd_vel(wheel_speed)

				# update message to publish to control/cmd_vel
				msg_cmd_vel.twist.linear.x = dr
				msg_cmd_vel.twist.angular.z = dtheta

				# publish message to topic
				#self.pub_cmd_vel.publish(msg_cmd_vel)

				"""

			# check if both are available
			if (not self.im[0] is None) and (not self.im[1] is None):

				# copy frames
				im = self.im				

				# clear both frames
				self.im = [None, None]

				# send to image processor
				ball = processImages(im)

				w = ball[0]
				rad = ball[1]

				l = w[0]
				r = w[2]

				print l, r, rad

				# convert to wheel speed (m/sec)
				k = 0.4
				d = r - l
				target_rad = 60
				drad = rad - target_rad
				krad = 0.002
				wheel_speed[0] = k * d + krad * drad
				wheel_speed[1] = k * -d + krad * drad
				
				# filter wheel speed
				gamma = 0.25
				for i in range(2):
					self.wheel_speed[i] += gamma * (wheel_speed[i] - self.wheel_speed[i])

				# convert wheel speed to command velocity (m/sec, Rad/sec)
				(dr, dtheta) = miro.utils.wheel_speed2cmd_vel(self.wheel_speed)

				# update message to publish to control/cmd_vel
				msg_cmd_vel.twist.linear.x = dr
				msg_cmd_vel.twist.angular.z = dtheta

				# publish message to topic
				self.pub_cmd_vel.publish(msg_cmd_vel)


			# yield
			time.sleep(0.01)
			self.t_now = self.t_now + 0.01

	def __init__(self, args):

		#Create object to convert ROS images to OpenCV format
		self.image_converter = CvBridge()		

		rospy.init_node("client_shepherd", anonymous=True)

		# state
		self.t_now = 0.0
		self.im = [None, None]
		self.wheel_speed = [0.0, 0.0]

		# inputs
		self.active = False
		self.package = None

		# handle args
		for arg in args:
			f = arg.find('=')
			if f == -1:
				key = arg
				val = ""
			else:
				key = arg[:f]
				val = arg[f+1:]
			if key == "pass":
				pass
			else:
				error("argument not recognised \"" + arg + "\"")

		# robot name
		topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

		# publish
		topic = topic_base_name + "/control/cmd_vel"
		print ("publish", topic)
		self.pub_cmd_vel = rospy.Publisher(topic, TwistStamped, queue_size=0)

		# subscribe
		topic = topic_base_name + "/sensors/package"
		print ("subscribe", topic)
		self.sub_package = rospy.Subscriber(topic, miro.msg.sensors_package, self.callback_package, queue_size=1, tcp_nodelay=True)

		self.sub_caml = rospy.Subscriber(topic_base_name + "/sensors/caml/compressed",
					CompressedImage, self.callback_caml)

		self.sub_camr = rospy.Subscriber(topic_base_name + "/sensors/camr/compressed",
					CompressedImage, self.callback_camr)

		# wait for connect
		print "wait for connect..."
		time.sleep(1)

		# set to active
		self.active = True

if __name__ == "__main__":

	# normal singular invocation
	main = controller(sys.argv[1:])
	main.loop()




