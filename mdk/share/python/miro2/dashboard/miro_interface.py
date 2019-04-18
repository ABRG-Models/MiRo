#!/home/dbuxton/mdk/share/python/miro2/dashboard/.venv/bin/python

# /usr/bin/env python
# MiRo-E ROS interfaces
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, UInt32, Int16MultiArray, String
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, BatteryState, Image, Imu, Range, CompressedImage

# Image handling
import cv2
from PIL import Image as Im
from PIL import ImageOps

# Other imports
import numpy as np
import miro2 as miro
import rospy


# TODO: Add definitions for physical or sim. robot
class MiroClient:
	def __init__(self):

		self.opt = {'Uncompressed': False}

		# Set topic root
		topic_root = "/miro"

		# TODO: Add mics (or is that just priw?)

		# Subscribe to ROS topics
		rospy.Subscriber(topic_root + '/core/affect/state', miro.msg.affect_state, self.callback_core_affect)
		rospy.Subscriber(topic_root + '/core/affect/time', UInt32, self.callback_core_time)

		rospy.Subscriber(topic_root + '/core/pril', Image, self.callback_pril)
		rospy.Subscriber(topic_root + '/core/prir', Image, self.callback_prir)
		rospy.Subscriber(topic_root + '/core/priw', Image, self.callback_priw)

		rospy.Subscriber(topic_root + '/core/detect_ball_l', UInt16MultiArray, self.callback_detect_ball_l)
		rospy.Subscriber(topic_root + '/core/detect_ball_r', UInt16MultiArray, self.callback_detect_ball_r)

		rospy.Subscriber(topic_root + '/core/selection/priority', Float32MultiArray, self.callback_selection_priority)
		rospy.Subscriber(topic_root + '/core/selection/inhibition', Float32MultiArray, self.callback_selection_inhibition)

		if self.opt['Uncompressed']:
			# TODO: Uncompressed callbacks not yet tested
			rospy.Subscriber(topic_root + '/sensors/caml', Image, self.callback_caml)
			rospy.Subscriber(topic_root + '/sensors/camr', Image, self.callback_camr)
		else:
			rospy.Subscriber(topic_root + '/sensors/caml/compressed', CompressedImage, self.callback_caml)
			rospy.Subscriber(topic_root + '/sensors/camr/compressed', CompressedImage, self.callback_camr)

		# Default data
		self.core_affect = None
		self.core_time = None
		self.core_pril = None
		self.core_prir = None
		self.core_priw = None
		self.core_detect_ball_l = None
		self.core_detect_ball_r = None
		self.selection_priority = None
		self.selection_inhibition = None
		self.sensors_caml = None
		self.sensors_camr = None

	def callback_core_affect(self, data):
		self.core_affect = data

	def callback_core_time(self, data):
		self.core_time = data

	def callback_detect_ball_l(self, data):
		self.core_detect_ball_l = data

	def callback_detect_ball_r(self, data):
		self.core_detect_ball_r = data

	def callback_selection_priority(self, data):
		self.selection_priority = data

	def callback_selection_inhibition(self, data):
		self.selection_inhibition = data

	# TODO: Image stitching before passing images back to dashboard
	def callback_caml(self, frame):
		self.sensors_caml = self.process_frame(frame)

	def callback_camr(self, frame):
		self.sensors_camr = self.process_frame(frame)

	def callback_pril(self, frame):
		self.core_pril = self.process_pri(frame)

	def callback_prir(self, frame):
		self.core_prir = self.process_pri(frame)

	def callback_priw(self, frame):
		self.core_priw = self.process_priw(frame)

	@staticmethod
	def process_frame(frame):
		# Decode image from numpy string format
		frame_bgr = cv2.imdecode(np.fromstring(frame.data, np.uint8), cv2.IMREAD_COLOR)

		# Convert image to RGB order
		frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_RGB2BGR)

		# Convert to image format
		return Im.fromarray(frame_rgb)

		# TODO: Can we convert directly from source in one go?
		# return Im.frombytes('RGB', (320, 176), np.fromstring(frame.data, np.uint8), 'raw')
		# return Im.frombytes('RGB', (182, 100), np.fromstring(frame.data, np.uint8), 'jpeg')

	@staticmethod
	def process_pri(frame):
		# Get monochrome image
		pri = Im.frombytes('L', (182, 100), np.fromstring(frame.data, np.uint8), 'raw')

		# TODO: Convert to image type with full alpha channel, make background transparent
		# Invert image for overlaying
		return ImageOps.invert(pri)

	@staticmethod
	def process_priw(frame):
		return Im.frombytes('L', (1, 256), np.fromstring(frame.data, np.uint8), 'raw')
