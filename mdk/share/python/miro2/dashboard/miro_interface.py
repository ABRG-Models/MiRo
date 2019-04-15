#!/home/dbuxton/mdk/share/python/miro2/dashboard/.venv/bin/python

# /usr/bin/env python
# MiRo-E ROS interfaces
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, UInt32, Int16MultiArray, String
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, BatteryState, Imu, Range, CompressedImage
import miro2 as miro
import rospy

# Image handling
import cv2
from PIL import Image

import threading
import time
import numpy as np

import Queue

# TODO: Add definitions for physical or sim. robot


class MiroClient:
	def __init__(self):

		# # set inactive
		# self.active = False

		# data = ros_data.get()

		self.opt = {'Uncompressed': False}

		uncompressed = False

		# Set topic root
		topic_root = "/miro"

		# Subscribe to ROS topics
		rospy.Subscriber(topic_root + "/core/affect", miro.msg.affect_state, self.callback_core_affect)

		if self.opt['Uncompressed']:
			# TODO: Uncompressed callbacks not yet tested
			rospy.Subscriber(topic_root + "/sensors/caml", Imu, self.callback_caml)
			rospy.Subscriber(topic_root + "/sensors/camr", Imu, self.callback_camr)
			#
			# rospy.Subscriber(topic_root + "/sensors/pril", Imu, self.callback_pril)
			# rospy.Subscriber(topic_root + "/sensors/prir", Imu, self.callback_prir)
			# rospy.Subscriber(topic_root + "/sensors/priw", Imu, self.callback_priw)
		else:
			rospy.Subscriber(topic_root + "/sensors/caml/compressed", CompressedImage, self.callback_caml)
			rospy.Subscriber(topic_root + "/sensors/camr/compressed", CompressedImage, self.callback_camr)
			#
			# rospy.Subscriber(topic_root + "/sensors/pril/compressed", CompressedImage, self.callback_pril)
			# rospy.Subscriber(topic_root + "/sensors/prir/compressed", CompressedImage, self.callback_prir)
			# rospy.Subscriber(topic_root + "/sensors/priw/compressed", CompressedImage, self.callback_priw)


		# self.sub_sensors = rospy.Subscriber(topic_root + "/sensors/package", miro.msg.sensors_package,
		# 									self.callback_platform_sensors)


		# # set active
		# self.active = True

		# Default data
		self.core_affect = None
		# self.core_affect = Queue.LifoQueue(maxsize=1)
		# self.core_affect.put(None)

		# self.sensors = {
		# 	'Cam L': None,
		# 	'Cam R': None
		# }

		self.sensors_caml = None
		self.sensors_camr = None
		# self.caml_fifo = fifo(self.opt['Uncompressed'])



	# TODO: Image stitching before passing images back to dashboard
	def callback_caml(self, frame):
		self.sensors_caml = self.process_frame(frame)

	def callback_camr(self, frame):
		self.sensors_camr = self.process_frame(frame)

	def callback_core_affect(self, data):
		self.core_affect = data
		# self.core_affect.put(data)

	@staticmethod
	def process_frame(frame):
		# Decode image from numpy string format
		frame_bgr = cv2.imdecode(np.fromstring(frame.data, np.uint8), cv2.IMREAD_COLOR)

		# Convert image to RGB order
		frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_RGB2BGR)

		# Convert to image format
		return Image.fromarray(frame_rgb)

		# TODO: Can we convert directly from source in one go?
		# return Image.frombytes('RGB', (320, 176), frame.data)


if __name__ == "__main__":
	miro_data = MiroClient()

	# while True:
	# 	if miro_data.core_affect is not None:
	# 		print(miro_data.core_affect)
	# 		time.sleep(1)

	# print(miro_data)