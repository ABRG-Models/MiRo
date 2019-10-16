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

import time
import sys
import os
import numpy as np

import rospy
import std_msgs
import geometry_msgs

import miro2 as miro

################################################################

def error(msg):

	print(msg)
	sys.exit(0)

def fmt(msg):

	x = msg.data
	s = "{0:.3f} {1:.3f}".format(x[0], x[1])
	return s

def fmt2(x):

	s = "{0:.3f} {1:.3f}".format(x[0], x[1])
	return s

def affectString(x):

	return "[ {0:.3f}, {1:.3f} ]".format(x.valence, x.arousal)

def sleepString(x):

	return "[ {0:.3f}, {1:.3f} ]".format(x.wakefulness, x.pressure)

################################################################

# if no argument provided
if len(sys.argv) != 2:

	# introduction
	print "this client interacts with the demo, which must be running."

	# show usage
	print "pass one of the following arguments to implement that test:"
	print "\tmood+"
	print "\tmood-"
	print "\temotion+"
	print "\temotion-"
	print "\tsleep+"
	print "\tsleep-"

	# done
	exit()

################################################################

class client:

	def callback_animal_state(self, msg):

		if self.show == "animal_state":
			print \
					"mood", affectString(msg.mood), \
					"emotion", affectString(msg.emotion), \
					"sleep", sleepString(msg.sleep)
			self.show = ""

	def callback_sensors(self, msg):

		pass

	def loop(self):

		# loop
		while not rospy.core.is_shutdown():

			# handle
			if self.test_to_run == "mood":
				msg = miro.msg.animal_adjust()
				# this is one way to adjust the state: see
				# adjust.msg file for more information
				msg.mood.valence.data = self.target
				msg.mood.valence.gamma = 0.025
				self.pub_animal_adjust.publish(msg)
				self.show = "animal_state"

			# handle
			if self.test_to_run == "emotion":
				msg = miro.msg.animal_adjust()
				# this is one way to adjust the state: see
				# adjust.msg file for more information
				msg.emotion.valence.data = self.target
				msg.emotion.valence.gamma = 0.025
				self.pub_animal_adjust.publish(msg)
				self.show = "animal_state"

			# handle
			if self.test_to_run == "sleep":
				msg = miro.msg.animal_adjust()
				# this is one way to adjust the state: see
				# adjust.msg file for more information
				msg.sleep.pressure.data = self.target
				msg.sleep.pressure.gamma = 0.025
				self.pub_animal_adjust.publish(msg)
				self.show = "animal_state"

			# state
			time.sleep(0.1)

	def __init__(self, test_to_run):

		# state
		self.show = ""
		self.target = 1.0
		self.test_to_run = test_to_run

		# handle
		if self.test_to_run == "emotion+":
			self.test_to_run = "emotion"
		elif self.test_to_run == "emotion-":
			self.test_to_run = "emotion"
			self.target = 0.0
		elif self.test_to_run == "mood+":
			self.test_to_run = "mood"
		elif self.test_to_run == "mood-":
			self.test_to_run = "mood"
			self.target = 0.0
		elif self.test_to_run == "sleep+":
			self.test_to_run = "sleep"
		elif self.test_to_run == "sleep-":
			self.test_to_run = "sleep"
			self.target = 0.0
		else:
			error("unrecognised argument")

		# robot name
		topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

		# subscribe
		topic = topic_base_name + "/sensors/package"
		print ("subscribe", topic)
		self.sub_sensors = rospy.Subscriber(topic, miro.msg.sensors_package, self.callback_sensors, queue_size=5, tcp_nodelay=True)

		# subscribe
		topic = topic_base_name + "/core/animal/state"
		print ("subscribe", topic)
		self.sub_animal_state = rospy.Subscriber(topic, miro.msg.animal_state, self.callback_animal_state, queue_size=5, tcp_nodelay=True)

		# publish
		topic = topic_base_name + "/core/animal/adjust"
		print ("publish", topic)
		self.pub_animal_adjust = rospy.Publisher(topic, miro.msg.animal_adjust, queue_size=0)

if __name__ == "__main__":

	rospy.init_node("client_demo", anonymous=True)
	main = client(sys.argv[1])
	main.loop()




