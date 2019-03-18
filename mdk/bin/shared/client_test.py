#!/usr/bin/python

import rospy
from std_msgs.msg import UInt8, UInt16, UInt32, Float32MultiArray, UInt16MultiArray, UInt32MultiArray
from nav_msgs.msg import Odometry

import geometry_msgs
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import JointState, Imu

import math
import numpy as np
import time
import sys
import os

import miro2 as miro

################################################################

def error(msg):

	print(msg)
	sys.exit(0)

def usage():

	print ("""
Usage:
	client_test.py wheels kin cos

	Without arguments, this help page is displayed. To run the
	client you must specify at least one option.

Options:
	wheels
		test the wheels (robot will move).

	kin
		test the kin(ematic) joints.

	cos
		test the cos(metic) joints.
	""")
	sys.exit(0)

def fmtflt(f):

	return "{:.3f}".format(f)

################################################################

class controller:

	def callback_kin(self, msg):

		# ignore until active
		if not self.active:
			return

		# report
		self.kin_sensor = msg.position

	def callback_package(self, msg):

		# ignore until active
		if not self.active:
			return

		# store
		self.sensors = msg

	def imp_report_wheels(self, msg_wheels):

		if self.report_wheels and not self.sensors is None:
			opto = self.sensors.wheel_speed_opto.data
			emf = self.sensors.wheel_speed_back_emf.data
			pwm = self.sensors.wheel_effort_pwm.data
			print \
				fmtflt(msg_wheels.twist.linear.x), \
				fmtflt(msg_wheels.twist.linear.x), \
				fmtflt(opto[0]), fmtflt(opto[1]), \
				fmtflt(emf[0]), fmtflt(emf[1]), \
				fmtflt(pwm[0]), fmtflt(pwm[1])

	def loop(self):

		# pars
		f_kin = 0.25
		f_cos = 1.0

		# state
		t_now = 0.0

		# message
		msg_kin = JointState()
		msg_kin.position = [0.0, np.radians(30.0), 0.0, 0.0]

		# message
		msg_wheels = TwistStamped()
		msg_push = miro.msg.push()

		# message
		msg_cos = Float32MultiArray()
		msg_cos.data = [0.5, 0.5, 0.0, 0.0, 0.5, 0.5]

		# message
		msg_illum = UInt32MultiArray()
		msg_illum.data = [0, 0, 0, 0, 0, 0]

		# loop
		while self.active and not rospy.core.is_shutdown():

			# break on loss of state file
			if not self.state_file is None:
				if not os.path.isfile(self.state_file):
					break

			# compute drive signals
			xk = math.sin(t_now * f_kin * 2 * math.pi)
			xc = math.sin(t_now * f_cos * 2 * math.pi)
			xc2 = math.sin(t_now * f_cos * 1 * math.pi)

			# feedback to user
			c = self.count % 10
			if c == 0 and self.report_input and not self.sensors is None:
				print "light", np.round(np.array(self.sensors.light.data) * 100.0)
				print "battery", np.round(np.array(self.sensors.battery.voltage) * 100.0) / 100.0
				print "touch_body", '{0:016b}'.format(self.sensors.touch_body.data)
				x = self.sensors.imu_head.linear_acceleration
				print "imu_head", [x.x, x.y, x.z]
				print "----------------------------------------------------------------"

			# send kin
			if len(self.kin):
				msg_kin.position[1] = np.radians(30.0)
				msg_kin.position[2] = np.radians(0.0)
				msg_kin.position[3] = np.radians(0.0)
				if "l" in self.kin:
					msg_kin.position[1] = xk * np.radians(20.0) + np.radians(30.0)
				if "y" in self.kin:
					t = xk * np.radians(45.0)
					if False:
						# this branch is used to measure YAW_COUNTS_PER_RAD
						t = (xk + 0.5) * np.radians(45.0)
						t = np.clip(t, 0.0, np.radians(45.0))
					msg_kin.position[2] = t
				if "p" in self.kin:
					msg_kin.position[3] = xk * np.radians(15.0) + np.radians(-7.0)
				self.pub_kin.publish(msg_kin)
				if not self.kin_sensor is None:
					a = "[{:.3f}".format(msg_kin.position[1]) + "=" + "{:.3f}]".format(self.kin_sensor[1])
					b = "[{:.3f}".format(msg_kin.position[2]) + "=" + "{:.3f}]".format(self.kin_sensor[2])
					c = "[{:.3f}".format(msg_kin.position[3]) + "=" + "{:.3f}]".format(self.kin_sensor[3])
					#print a, b, c

			# send cos
			if len(self.cos):
				sc = 0.5
				if "h" in self.cos:
					for i in range(2, 6):
						msg_cos.data[i] = xc * sc + 0.5
				if "l" in self.cos:
					for i in [2, 4]:
						msg_cos.data[i] = xc * sc + 0.5
				if "r" in self.cos:
					for i in [3, 5]:
						msg_cos.data[i] = xc * sc + 0.5
				if "y" in self.cos:
					for i in [2, 3]:
						msg_cos.data[i] = xc * sc + 0.5
				if "e" in self.cos:
					for i in [4, 5]:
						msg_cos.data[i] = xc * sc + 0.5
				if "w" in self.cos:
					msg_cos.data[1] = xc * 0.5 + 0.5
				if "d" in self.cos:
					msg_cos.data[0] = xc * 0.5 + 0.5
				if "x" in self.cos:
					if xc2 >= 0:
						msg_cos.data[1] = xc * 0.5 + 0.5
					else:
						msg_cos.data[0] = xc * 0.5 + 0.5
				self.pub_cos.publish(msg_cos)

			# send wheels
			if not self.wheels is None:
				v = 0.0
				Tq = 0.2
				T = 2.0
				t1 = Tq
				t2 = t1 + T
				t3 = t2 + T
				t4 = t3 + Tq
				if t_now < t1:
					v = 0.0
				elif t_now < t2:
					v = (t_now - t1) / T
				elif self.forever:
					v = 1.0
				elif t_now < t3:
					v = 1.0 - (t_now - t2) / T
				elif t_now < t4:
					v = 0.0
				else:
					self.active = False
				msg_wheels.twist.linear.x = v * self.wheels
				msg_wheels.twist.angular.z = 0.0
				self.pub_wheels.publish(msg_wheels)
				self.imp_report_wheels(msg_wheels)

			# send wheels
			if self.wheelsf:
				msg_wheels.twist.linear.x = 0.4
				msg_wheels.twist.angular.z = 0.0
				self.pub_wheels.publish(msg_wheels)
				self.imp_report_wheels(msg_wheels)

			# send wheels
			if not self.spin is None:
				v = 0.0
				Tq = 0.2
				T = 1.0
				t1 = Tq
				t2 = t1 + T
				t3 = t2 + T
				t4 = t3 + Tq
				if t_now < t1:
					v = 0.0
				elif t_now < t2:
					v = (t_now - t1) / T
				elif self.forever:
					v = 1.0
				elif t_now < t3:
					v = 1.0 - (t_now - t2) / T
				elif t_now < t4:
					v = 0.0
				else:
					self.active = False
				msg_wheels.twist.linear.x = 0.0
				msg_wheels.twist.angular.z = v * 6.2832 * self.spin
				self.pub_wheels.publish(msg_wheels)
				self.imp_report_wheels(msg_wheels)

			# send push
			if self.push:
				msg_push.link = miro.constants.LINK_HEAD
				msg_push.flags = miro.constants.PUSH_FLAG_VELOCITY
				msg_push.pushpos = geometry_msgs.msg.Vector3(miro.constants.LOC_NOSE_TIP_X, miro.constants.LOC_NOSE_TIP_Y, miro.constants.LOC_NOSE_TIP_Z)
				msg_push.pushvec = geometry_msgs.msg.Vector3(0.0, 0.2 * xk, 0.0)
				self.pub_push.publish(msg_push)

			# send illum
			q = int(xc * 127 + 128)
			for i in range(0, 3):
				msg_illum.data[i] = (q << ((2-i) * 8)) | 0xFF000000
			for i in range(3, 6):
				msg_illum.data[i] = (q << ((i-3) * 8)) | 0xFF000000
			self.pub_illum.publish(msg_illum)

			# send P1P
			if not self.P1P is None:
				msg_platform = UInt32()
				msg_platform.data = 0
				if self.P1P == 'x':
					self.active = False
				else:
					if self.P1P == 'P':
						msg_platform.data = 1024
					elif self.P1P == 'O':
						msg_platform.data = 2048
					elif self.P1P == 'I':
						msg_platform.data = 2
					self.pub_platform.publish(msg_platform)
					self.P1P = 'x'

			# state
			time.sleep(0.02)
			self.count = self.count + 1
			t_now = t_now + 0.02

	def __init__(self, args):

		rospy.init_node("client_test", anonymous=True)

		# state
		self.state_file = None
		self.count = 0
		self.active = False
		self.forever = False

		# input
		self.report_input = True
		self.sensors = None

		self.kin_sensor = None

		# options
		self.report_wheels = False
		self.wheels = None
		self.wheelsf = False
		self.spin = None
		self.kin = ""
		self.cos = ""
		self.P1P = None
		self.push = False

		# handle args
		for arg in args:
			f = arg.find('=')
			if f == -1:
				key = arg
				val = ""
			else:
				key = arg[:f]
				val = arg[f+1:]
			if key == "state_file":
				self.state_file = val
			elif key == "forever":
				self.forever = True
			elif key == "wheels":
				self.wheels = 0.4
			elif key == "stall":
				self.wheels = 4.0
				self.report_wheels = True
				self.report_input = False
			elif key == "stall-":
				self.wheels = -4.0
				self.report_wheels = True
				self.report_input = False
			elif key == "wheels-":
				self.wheels = -0.4
			elif key == "wheelsf":
				self.wheelsf = True
				self.report_wheels = True
				self.report_input = False
			elif key == "spin":
				self.spin = 1.0
			elif key == "spin-":
				self.spin = -1.0
			elif key == "wheels_cont":
				self.wheels = 0.4
				self.report_wheels = True
				self.report_input = False
			elif key == "spin_cont":
				self.spin = 1.0
				self.report_wheels = True
				self.report_input = False
			elif key == "push":
				self.push = True
			elif key == "kin":
				self.kin = "lyp"
				self.report_input = False
			elif key == "lift":
				self.kin = "l"
				self.report_input = False
			elif key == "yaw":
				self.kin = "y"
				self.report_input = False
			elif key == "pitch":
				self.kin = "p"
				self.report_input = False
			elif key == "cos":
				self.cos = "hw"
			elif key == "cosl":
				self.cos = "l"
			elif key == "cosr":
				self.cos = "r"
			elif key == "eyes":
				self.cos = "y"
			elif key == "ears":
				self.cos = "e"
			elif key == "wag":
				self.cos = "w"
			elif key == "droop":
				self.cos = "d"
			elif key == "wagdroop":
				self.cos = "x"
			elif key == "P1P":
				self.P1P = val
			elif key == "quiet":
				self.report_input = False
			elif key == "workout":
				self.cos = "lrx"
				self.kin = "lyp"
			else:
				error("argument not recognised \"" + arg + "\"")

		# robot name
		topic_base = "/" + os.getenv("MIRO_ROBOT_NAME") + "/"

		# publish
		topic = topic_base + "control/cmd_vel"
		print ("publish", topic)
		self.pub_wheels = rospy.Publisher(topic, TwistStamped, queue_size=0)

		# publish
		topic = topic_base + "control/kinematic_joints"
		print ("publish", topic)
		self.pub_kin = rospy.Publisher(topic, JointState, queue_size=0)

		# publish
		topic = topic_base + "control/cosmetic_joints"
		print ("publish", topic)
		self.pub_cos = rospy.Publisher(topic, Float32MultiArray, queue_size=0)

		# publish
		topic = topic_base + "control/illum"
		print ("publish", topic)
		self.pub_illum = rospy.Publisher(topic, UInt32MultiArray, queue_size=0)

		# publish
		topic = topic_base + "core/mpg/push"
		print ("publish", topic)
		self.pub_push = rospy.Publisher(topic, miro.msg.push, queue_size=0)

		# publish
		topic = topic_base + "control/platform"
		print ("publish", topic)
		self.pub_platform = rospy.Publisher(topic, UInt32, queue_size=0)

		# subscribe
		topic = topic_base + "sensors/package"
		print ("subscribe", topic)
		self.sub_package = rospy.Subscriber(topic, miro.msg.sensors_package, self.callback_package)

		# wait for connect
		print "wait for connect..."
		time.sleep(1)

		# set to active
		self.active = True

if __name__ == "__main__":

	"""
	if len(sys.argv) >= 2 and sys.argv[1] == "governed":

		state_file = os.getenv("MIRO_DIR_STATE") + "/client_test"

		while True:

			# wait for state file
			if not os.path.isfile(state_file):
				print "."
				time.sleep(0.1)
				continue
			# read state file
			with open(state_file) as f:
				cmd = f.read().strip()
			# split to args
			args = cmd.split(' ')
			# run controller
			main = controller(args, state_file)
			main.loop()
	"""

	# normal singular invocation
	main = controller(sys.argv[1:])
	main.loop()




