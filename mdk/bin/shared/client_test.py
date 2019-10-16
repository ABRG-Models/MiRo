#!/usr/bin/python
#
#	@section COPYRIGHT
#	Copyright (C) 2019 Consequential Robotics Ltd
#	
#	@section AUTHOR
#	Consequential Robotics http://consequentialrobotics.com
#	
#	@section LICENSE
#	For a full copy of the license agreement, and a complete
#	definition of "The Software", see LICENSE in the MDK root
#	directory.
#	
#	Subject to the terms of this Agreement, Consequential
#	Robotics grants to you a limited, non-exclusive, non-
#	transferable license, without right to sub-license, to use
#	"The Software" in accordance with this Agreement and any
#	other written agreement with Consequential Robotics.
#	Consequential Robotics does not transfer the title of "The
#	Software" to you; the license granted to you is not a sale.
#	This agreement is a binding legal agreement between
#	Consequential Robotics and the purchasers or users of "The
#	Software".
#	
#	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY
#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#	

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

# constants
max_fwd_spd = 0.4
T_ramp_up_down = 2.0

################################################################

def error(msg):

	print(msg)
	sys.exit(0)

def usage():

	print ("""
Usage:
	client_test.py <options>

	Without arguments, this help page is displayed. To run the
	client you must specify at least one option.

Options:
	listen
		just listen

	wheels (wheels- for reverse)
		move forward (robot will move).

	spin (spin- for reverse)
		turn on the spot (robot will move).

	stall (stall- for reverse)
		test wheels and show additional data for stall analysis.

	wheelsf (wheelsf- for reverse)
		hold the wheels at maximum forward speed indefinitely.

	push
		test the push control channel.

	kin (lift, yaw, pitch)
		test all the kin(ematic) joints (or just one).

	cos (cosl, cosr, eyes, ears, wag, droop, wagdroop)
		test all the cos(metic) joints (or just a subset).

	illum
		exercise the LED illumination arrays.

	workout
		test all the kin and cos joints together.

	forever
		keep going until user hits CTRL+C

	--quiet
		be quiet

	--no-cliff-reflex
		disable robot's cliff reflex before beginning

	--no-illum
		disable illumination LEDs under shell

	--status-leds
		enable status LEDs on PCBs (won't work if locked)

	--exit
		exit immediately after setting flags, and make
		flags persistent after exit
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
			msg = fmtflt(msg_wheels.twist.linear.x) + " " + \
				fmtflt(msg_wheels.twist.angular.z) + " " + \
				fmtflt(opto[0]) + " " + fmtflt(opto[1]) + " " + \
				fmtflt(emf[0]) + " " + fmtflt(emf[1]) + " " + \
				fmtflt(pwm[0]) + " " + fmtflt(pwm[1])
			print msg
			if not self.report_file is None:
				self.report_file += msg + "\n"

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
			xcc = math.cos(t_now * f_cos * 2 * math.pi)
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
				T = T_ramp_up_down
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
			if not self.wheelsf is None:
				msg_wheels.twist.linear.x = self.wheelsf
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
			if self.illum:
				q = int(xcc * -127 + 128)
				if t_now >= 4.0:
					self.active = False
					q = 0
				for i in range(0, 3):
					msg_illum.data[i] = (q << ((2-i) * 8)) | 0xFF000000
				for i in range(3, 6):
					msg_illum.data[i] = (q << ((i-3) * 8)) | 0xFF000000
				self.pub_illum.publish(msg_illum)

			# state
			time.sleep(0.02)
			self.count = self.count + 1
			t_now = t_now + 0.02

		# end loop
		if not self.report_file is None:
			with open('/tmp/client_test.report_file', 'w') as file:
				file.write(self.report_file)

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
		self.report_file = None
		self.wheels = None
		self.wheelsf = None
		self.spin = None
		self.kin = ""
		self.cos = ""
		self.illum = False
		self.push = False
		self.opts = []

		# must have one option
		if len(args) == 0:
			usage()
			sys.exit(0)

		# handle args
		for arg in args:

			# collect opts
			if arg[0:2] == '--':
				self.opts.append(arg)
				continue

			f = arg.find('=')
			if f == -1:
				key = arg
				val = ""
			else:
				key = arg[:f]
				val = arg[f+1:]
			if key == "listen":
				# allow use just to show output
				pass
			elif key == "--no-cliff-reflex":
				self.opts.append(key)
			elif key == "state_file":
				self.state_file = val
			elif key == "forever":
				self.forever = True
			elif key == "wheels":
				self.wheels = max_fwd_spd
			elif key == "wheels-":
				self.wheels = -max_fwd_spd
			elif key == "wheelsc":
				self.wheels = max_fwd_spd
				self.report_wheels = True
				self.report_input = False
				self.report_file = ""
			elif key == "wheelsc-":
				self.wheels = -max_fwd_spd
				self.report_wheels = True
				self.report_input = False
				self.report_file = ""
			elif key == "wheelsf":
				self.wheelsf = max_fwd_spd
				self.report_wheels = True
				self.report_input = False
				self.report_file = ""
			elif key == "wheelsf-":
				self.wheelsf = -max_fwd_spd
				self.report_wheels = True
				self.report_input = False
				self.report_file = ""
			elif key == "stall":
				self.wheels = 4.0
				self.report_wheels = True
				self.report_input = False
				self.report_file = ""
			elif key == "stall-":
				self.wheels = -4.0
				self.report_wheels = True
				self.report_input = False
				self.report_file = ""
			elif key == "spin":
				self.spin = 1.0
			elif key == "spin-":
				self.spin = -1.0
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
			elif key == "illum":
				self.illum = True
			elif key == "workout":
				self.cos = "lrx"
				self.kin = "lyp"
			else:
				error("argument not recognised \"" + arg + "\"")

		# handle opts
		if "--quiet" in self.opts:
			self.report_input = False

		# robot name
		topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

		# publish
		topic = topic_base_name + "/control/cmd_vel"
		print ("publish", topic)
		self.pub_wheels = rospy.Publisher(topic, TwistStamped, queue_size=0)

		# publish
		topic = topic_base_name + "/control/kinematic_joints"
		print ("publish", topic)
		self.pub_kin = rospy.Publisher(topic, JointState, queue_size=0)

		# publish
		topic = topic_base_name + "/control/cosmetic_joints"
		print ("publish", topic)
		self.pub_cos = rospy.Publisher(topic, Float32MultiArray, queue_size=0)

		# publish
		topic = topic_base_name + "/control/illum"
		print ("publish", topic)
		self.pub_illum = rospy.Publisher(topic, UInt32MultiArray, queue_size=0)

		# publish
		topic = topic_base_name + "/core/mpg/push"
		print ("publish", topic)
		self.pub_push = rospy.Publisher(topic, miro.msg.push, queue_size=0)

		# publish
		topic = topic_base_name + "/control/flags"
		print ("publish", topic)
		self.pub_flags = rospy.Publisher(topic, UInt32, queue_size=0)

		# subscribe
		topic = topic_base_name + "/sensors/package"
		print ("subscribe", topic)
		self.sub_package = rospy.Subscriber(topic, miro.msg.sensors_package, self.callback_package, queue_size=5, tcp_nodelay=True)

		# wait for connect
		print "wait for connect..."
		time.sleep(1)

		# send control flags
		default_flags = miro.constants.PLATFORM_D_FLAG_DISABLE_STATUS_LEDS
		msg = UInt32()
		msg.data = default_flags
		if "--exit" in self.opts:
			msg.data |= miro.constants.PLATFORM_D_FLAG_PERSISTENT
		if "--no-cliff-reflex" in self.opts:
			msg.data |= miro.constants.PLATFORM_D_FLAG_DISABLE_CLIFF_REFLEX
		if "--no-illum" in self.opts:
			msg.data |= miro.constants.PLATFORM_D_FLAG_DISABLE_ILLUM
		if "--status-leds" in self.opts:
			msg.data &= ~(miro.constants.PLATFORM_D_FLAG_DISABLE_STATUS_LEDS)
		print "send control flags... ",
		print hex(msg.data),
		self.pub_flags.publish(msg)
		print "OK"

		# and exit
		if "--exit" in self.opts:
			print "exit after setting flags..."
			print "(NB: flags will be reset if you run this client again)"
			exit()

		# set to active
		self.active = True

if __name__ == "__main__":

	# normal singular invocation
	main = controller(sys.argv[1:])
	main.loop()
