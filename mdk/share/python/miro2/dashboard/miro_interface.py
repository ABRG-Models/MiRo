#!/home/dbuxton/mdk/share/python/miro2/dashboard/.venv/bin/python

# /usr/bin/env python
import rospy

# MiRo-E ROS interfaces
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, UInt32, Int16MultiArray, String
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, BatteryState, Imu, Range, CompressedImage
import miro2 as miro

import cv2
from PIL import Image

import threading
import time
import numpy as np

import Queue


# class fifo:
#
#     def __init__(self, uncompressed):
#         self.N = 1  # raise if we get "overflow in fifo", or to improve fps measurement
#         self.buf = [None] * self.N
#         self.r = 0
#         self.w = 0
#         self.tN = 30
#         self.ti = 0
#         self.td = 10
#         self.tt = [None] * self.tN
#         self.hz = None
#         self.uncompressed = uncompressed
#         self.lock = threading.Lock()
#
#     def push(self, frm):
#         self.lock.acquire()
#         try:
#             t_frm = time.time()
#             if self.buf[self.w] is None:
#                 if self.uncompressed:
#                     pb = GdkPixbuf.Pixbuf.new_from_data(frm.data,
#                         GdkPixbuf.Colorspace.RGB, False, 8,
#                         frm.width, frm.height, frm.step)
#                 else:
#                     im = cv2.imdecode(np.fromstring(frm.data, np.uint8),
#                                 cv2.IMREAD_COLOR)
#                     w = im.shape[1]
#                     h = im.shape[0]
#                     N = h * w
#                     for i in range(0, N):
#                         tmp = im.data[i*3+0]
#                         im.data[i*3+0] = im.data[i*3+2]
#                         im.data[i*3+2] = tmp
#                     pb = im
#                     #pb = GdkPixbuf.Pixbuf.new_from_data(im.data, GdkPixbuf.Colorspace.RGB, False, 8, w, h, w*3)
#
#                 self.buf[self.w] = pb
#             else:
#                 #print("**** frame dropped ****")
#                 pass
#             self.tt[self.ti] = t_frm
#             ti_bak = self.ti - self.td
#             if ti_bak < 0:
#                 ti_bak = ti_bak + self.tN
#             if not self.tt[ti_bak] is None:
#                 dt = t_frm - self.tt[ti_bak]
#                 self.hz = self.td / dt
#                 if self.hz > (self.td + 3) and self.td < (self.tN - 1):
#                     self.td = self.td + 1
#                     #print "> ", self.td
#                 if self.hz < (self.td - 3) and self.td > 5:
#                     self.td = self.td - 1
#                     #print "< ", self.td
#             self.ti = (self.ti + 1) % self.tN
#         finally:
#             self.lock.release()
#
#     def pop(self):
#         obj = self.buf[self.r]
#         if not obj is None:
#             self.buf[self.r] = None
#             self.r = (self.r + 1) % self.N
#         return obj
#
#     def latest(self):
#         ret = None
#         while True:
#             obj = self.pop()
#             if obj is None:
#                 break
#             ret = obj
#         return ret
#
#     def freq(self):
#         hz = self.hz
#         if not hz is None:
#             self.hz = None
#         return hz



# TODO: Add definitions for physical or sim. robot
class MiroClient:

	# def callback_platform_sensors(self, object):
	# 	# # ignore until active
	# 	# if not self.active:
	# 	# 	return
	#
	# 	# store object
	# 	self.platform_sensors = object
	# 	print(self.platform_sensors)

	@staticmethod
	def process_frame(frame):
		# Decode image from numpy string format
		frame_bgr = cv2.imdecode(np.fromstring(frame.data, np.uint8), cv2.IMREAD_COLOR)

		# Convert image to RGB order
		frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_RGB2BGR)

		# Convert to image format
		return Image.fromarray(frame_rgb)

		# FIXME: Can we convert directly from source in one go?
		# return Image.frombytes('RGB', (320, 176), frame.data)

	# TODO: Image stitching before passing images back to dashboard
	def callback_caml(self, frame):
		self.sensors_caml = self.process_frame(frame)

	def callback_camr(self, frame):
		self.sensors_camr = self.process_frame(frame)

	def callback_core_affect(self, data):
		self.core_affect = data
		# self.core_affect.put(data)


	def __init__(self):

		# # set inactive
		# self.active = False

		# data = ros_data.get()

		self.opt = {'Uncompressed': False}

		uncompressed = False

		topic_root = "/miro"

		# rospy.init_node('listener', anonymous=True, disable_rostime=True)

		rospy.Subscriber(topic_root + "/core/affect", miro.msg.affect_state, self.callback_core_affect)

		if self.opt['Uncompressed']:
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

		# '''
		# self.sub_rgbl = rospy.Subscriber(topic_root + "/core/rgbl/compressed", CompressedImage, self.callback_rgbl)
		# self.sub_rgbr = rospy.Subscriber(topic_root + "/core/rgbr/compressed", CompressedImage, self.callback_rgbr)
		# '''


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


if __name__ == "__main__":
	miro_data = MiroClient()

	# while True:
	# 	if miro_data.core_affect is not None:
	# 		print(miro_data.core_affect)
	# 		time.sleep(1)

	# print(miro_data)