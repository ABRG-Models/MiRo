#!/usr/bin/python -u
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
#
# This client funnels all the information needed by
# the web client into one package, for simple delivery.

import rospy
import std_msgs

import math
import numpy as np
import time
import sys
import os

import miro2 as miro

DECIMATE_FACTOR=5

mics = []

################################################################

def error(msg):

	print(msg)
	sys.exit(0)

################################################################

class controller:

	def subscribe(self, enable):

		if enable:
			# subscribe
			topic = self.topic_base_name + "/sensors/package"
			print ("subscribe", topic)
			self.sub_package = rospy.Subscriber(topic, miro.msg.sensors_package, self.callback_package, queue_size=1, tcp_nodelay=True)

			# subscribe
			topic = self.topic_base_name + "/sensors/touch_body"
			print ("subscribe", topic)
			self.sub_touch_body = rospy.Subscriber(topic, std_msgs.msg.UInt16, self.callback_touch_body, queue_size=1, tcp_nodelay=True)

			# subscribe
			topic = self.topic_base_name + "/sensors/touch_head"
			print ("subscribe", topic)
			self.sub_touch_head = rospy.Subscriber(topic, std_msgs.msg.UInt16, self.callback_touch_head, queue_size=1, tcp_nodelay=True)

			# subscribe
			topic = self.topic_base_name + "/control/illum"
			print ("subscribe", topic)
			self.sub_illum = rospy.Subscriber(topic, std_msgs.msg.UInt32MultiArray, self.callback_illum, queue_size=1, tcp_nodelay=True)

			# subscribe
			topic = self.topic_base_name + "/sensors/mics"
			print ("subscribe", topic)
			self.sub_mics = rospy.Subscriber(topic, std_msgs.msg.Int16MultiArray, self.callback_mics, queue_size=1, tcp_nodelay=True)

			# subscribe
			topic = self.topic_base_name + "/control/tone"
			print ("subscribe", topic)
			self.sub_tone = rospy.Subscriber(topic, std_msgs.msg.UInt16MultiArray, self.callback_tone, queue_size=1, tcp_nodelay=True)

			# mark
			self.subscribed = True

		else:
			# unsubscribe
			print ("unsubscribe from all...")
			self.sub_package.unregister()
			self.sub_touch_body.unregister()
			self.sub_touch_head.unregister()
			self.sub_illum.unregister()
			self.sub_mics.unregister()
			self.sub_tone.unregister()

			# mark
			self.subscribed = False

	def callback_touch_body(self, msg):
	
		self.touch[0] |= msg.data

	def callback_touch_head(self, msg):
	
		self.touch[1] |= msg.data
		
	def callback_illum(self, msg):
	
		self.illum = msg.data

	def callback_mics(self, msg):

		data = np.array(msg.data).astype('float32') * (1.0 / 32768.0)
		rms_l = np.sqrt(np.mean(data[0:500]**2))
		rms_r = np.sqrt(np.mean(data[500:1000]**2))
		self.audio_level = np.max((self.audio_level, np.mean([rms_l, rms_r])))

		# save mics
		#mics.append(msg.data)
		
	def callback_tone(self, msg):
	
		if self.tone is None:
			self.tone = msg.data

	def callback_package(self, msg):

		# ignore until active
		if not self.active:
			return

		# decimate
		self.decimate += 1
		if self.decimate < DECIMATE_FACTOR:
			return
		self.decimate = 0

		# store
		self.package = msg

	def loop(self):

		# loop
		while self.active and not rospy.core.is_shutdown():

			# if sensor data has arrived
			if not self.package is None:

				# transfer
				self.msg_funnel_web.cliff = self.package.cliff.data
				self.msg_funnel_web.light = self.package.light.data
				self.msg_funnel_web.sonar = self.package.sonar.range
				self.msg_funnel_web.touch = self.touch
				self.msg_funnel_web.illum = self.illum
				self.msg_funnel_web.audio_level = self.audio_level
				if self.tone is None:
					self.msg_funnel_web.tone = [0, 0, 0]
				else:
					self.msg_funnel_web.tone = self.tone
					self.tone = None

				# publish message to topic
				self.pub_funnel_web.publish(self.msg_funnel_web)

				# clear data that has now been used
				self.package = None
				self.touch = [0, 0]
				self.audio_level = 0

			# yield
			if self.subscribed:
				time.sleep(0.02)
				self.t_now = self.t_now + 0.02
			else:
				#print "not listened at time", self.t_now
				time.sleep(1.0)
				self.t_now = self.t_now + 1.0

			# test if anyone is subscribed
			listened = self.pub_funnel_web.get_num_connections() > 0
			if listened and not self.subscribed:
				self.subscribe(True)
			if not listened and self.subscribed:
				self.subscribe(False)

		# finalise report
		print "\n\nexit..."
		print " "

		# save mics
		#np.savetxt('/tmp/mics', mics, fmt='%.3f', delimiter=' ')

	def __init__(self, args):

		rospy.init_node("client_funnel_web", anonymous=True)

		# state
		self.active = False
		self.subscribed = False
		self.t_now = 0.0
		self.decimate = 0
		self.touch = [0, 0]
		self.illum = [0, 0, 0, 0, 0, 0]
		self.audio_level = 0
		self.tone = None

		# inputs
		self.package = None

		# robot name
		self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

		# publish
		topic = self.topic_base_name + "/funnel_web"
		print ("publish", topic)
		self.pub_funnel_web = rospy.Publisher(topic, miro.msg.funnel_web, queue_size=0)
		self.msg_funnel_web = miro.msg.funnel_web()

		# wait for connect
		print "wait for connect... ",
		sys.stdout.flush()
		time.sleep(1)
		print "OK"

		# set to active
		self.active = True

if __name__ == "__main__":

	# normal singular invocation
	main = controller(sys.argv[1:])
	main.loop()




