#!/usr/bin/python

import rospy
from std_msgs.msg import UInt8, UInt16, UInt32, Float32MultiArray, UInt16MultiArray, UInt32MultiArray
from nav_msgs.msg import Odometry

import geometry_msgs
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import JointState, Imu, CompressedImage

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
				print "cliff", np.round(np.array(self.sensors.cliff.data) * 15.0)
				print "battery", np.round(np.array(self.sensors.battery.voltage) * 100.0) / 100.0
				print "touch_body", '{0:016b}'.format(self.sensors.touch_body.data)
				x = self.sensors.imu_head.linear_acceleration
				print "imu_head", [x.x, x.y, x.z]
				print "----------------------------------------------------------------"

			# send wheels
			if self.shoot:
				msg_kin.position[1] = np.radians(0.0)
				msg_kin.position[2] = np.radians(0.0)
				msg_kin.position[3] = np.radians(0.0)
				self.pub_kin.publish(msg_kin)

				v = 0.0
				Tq = 0.1
				T = 1.0
				t1 = Tq
				t2 = t1 + T
				t3 = t2 + T
				t4 = t3 + Tq

				if t_now < t1:
					v = 0.0
				elif t_now < t2:
					v = (t_now - t1) / T
				elif t_now < t3:
					v = 0.5 - (t_now - t2) / T
				elif t_now < t4:
					v = 0.0
				else:
					self.active = False
				msg_wheels.twist.linear.x = v * 4.0
				msg_wheels.twist.angular.z = 0.0
				self.pub_wheels.publish(msg_wheels)
				self.imp_report_wheels(msg_wheels)

			if len(self.toss): 
				if "l" in self.toss:
					t = xk * np.radians(55.0)
					v = 0
					Tq = 0.1
					T = 1.0
					t1 = Tq
					t2 = t1 + T
					t3 = t2 + T
					t4 = t3 + Tq
					if t_now < t1:
						v = 0.0
					elif t_now < t2:
						v = (t_now - t1) / T
					elif t_now < t3:
						v = 0.3 - (t_now - t2*0.6) / T
					elif t_now < t4:
						v = 0.0
					else:
						self.active = False
					msg_wheels.twist.angular.z = v * -3.0
					msg_wheels.twist.linear.x = 0

					msg_kin.position[1] = np.radians(75.0)
					msg_kin.position[2] = -t
					msg_kin.position[3] = np.radians(30.0)
					self.pub_kin.publish(msg_kin)
					self.imp_report_wheels(msg_wheels)
					self.pub_wheels.publish(msg_wheels)

				if "r" in self.toss:
					t = xk * np.radians(55.0)
					v = 0
					Tq = 0.1
					T = 1.0
					t1 = Tq
					t2 = t1 + T
					t3 = t2 + T
					t4 = t3 + Tq
					if t_now < t1:
						v = 0.0
					elif t_now < t2:
						v = (t_now - t1) / T
					elif t_now < t3:
						v = 0.3 - (t_now - t2*0.6) / T
					elif t_now < t4:
						v = 0.0
					else:
						self.active = False
					msg_wheels.twist.angular.z = v * 3.0
					msg_wheels.twist.linear.x = 0

					msg_kin.position[1] = np.radians(75.0)
					msg_kin.position[2] = t
					msg_kin.position[3] = np.radians(30.0)
					self.pub_kin.publish(msg_kin)
					self.imp_report_wheels(msg_wheels)
					self.pub_wheels.publish(msg_wheels)

			if self.stopb:
				msg_kin.position[1] = np.radians(70.0)
				msg_kin.position[2] = np.radians(0.0)
				msg_kin.position[3] = np.radians(-20.0)
				self.pub_kin.publish(msg_kin)

			if self.ready:
				msg_kin.position[1] = np.radians(0.0)
				msg_kin.position[2] = np.radians(0.0)
				msg_kin.position[3] = np.radians(0.0)
				self.pub_kin.publish(msg_kin)

			if self.dribble and not self.sensors is None:

				msg_wheels.twist.linear.x = 0.2
				msg_wheels.twist.angular.z = 0.0
				self.pub_wheels.publish(msg_wheels)
				self.imp_report_wheels(msg_wheels)

				if self.sensors.cliff.data[0] * 15.0 != 15.0:
					msg_wheels.twist.angular.z = -1.0
					self.pub_wheels.publish(msg_wheels)
					self.imp_report_wheels(msg_wheels)
				elif self.sensors.light.data[1] * 15.0 != 15.0:
					msg_wheels.twist.angular.z = 1.0
					self.pub_wheels.publish(msg_wheels)
					self.imp_report_wheels(msg_wheels)

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
		self.shoot = False
		self.toss = ""
		self.stopb = False
		self.dribble = False
		self.ready = False


		# handle args
		for arg in args:
			f = arg.find('=')
			if f == -1:
				key = arg
				val = ""
			else:
				key = arg[:f]
				val = arg[f+1:]
			if key == "shoot":
				self.shoot = True
			elif key == "passl":
				self.toss = "l"
			elif key == "passr":
				self.toss = "r"
			elif key == "stopb":
				self.stopb = True
			elif key == "dribble":
				self.dribble = True
			elif key == "ready":
				self.ready = True
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

#		# subscribe
#		topic = topic_base + "sensors/caml/compressed"
#		print ("subscribe", topic)
#		self.sub_caml = rospy.Subscriber(topic,
#					CompressedImage, self.callback_caml)


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




