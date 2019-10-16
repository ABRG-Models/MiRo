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

################################################################

# if no argument provided
if len(sys.argv) != 2:

	# show usage
	print "pass one of the following arguments to show that signal:"
	print "\tcliff"
	print "\twheels"
	print "\tmics"
	print "\tcmd_vel"
	print "\tlog"

	# done
	exit()
	
################################################################

class client:

	def callback_sensors(self, msg):

		# show topic
		if self.topic_to_show == "cliff":
		
			# show data
			print (np.array([msg.cliff.data[0], msg.cliff.data[1]]) * 15.0).astype('int')
			
		# show topic
		if self.topic_to_show == "wheels":
			
			# extract data to show
			x = msg.wheel_speed_cmd.data
			
			# show only when movement is commanded?
			show = x[0] != 0.0 or x[1] != 0.0
			
			# or, show always
			show = True

			# if show now
			if show:
				self.wait = False
				print \
					fmt(msg.wheel_speed_cmd), \
					fmt(msg.wheel_speed_opto), \
					fmt(msg.wheel_speed_back_emf), \
					fmt(msg.wheel_effort_pwm)
			else:
				if not self.wait:
					print "________________________________________________________________\n"
				self.wait = True

	def callback_mics(self, msg):

		# show topic
		if self.topic_to_show == "mics":
		
			# show data
			mag = (np.mean(np.abs(np.array(msg.data))) / 32768.0 * 100.0).astype('int')
			print "received", len(msg.data), "samples with mean mag", mag, "%"

	def callback_cmd_vel(self, msg):

		# show topic
		if self.topic_to_show == "cmd_vel":
		
			# extract data to show
			x = [msg.twist.linear.x, msg.twist.angular.z]
			
			# show only when movement is commanded?
			show = x[0] != 0.0 or x[1] != 0.0
			
			# or, show always
			#show = True
		
			# show data
			if show:
				self.wait = False
				print fmt2(x)

			else:
				if not self.wait:
					print "________________________________________________________________\n"
				self.wait = True

	def callback_log(self, msg):

		# show topic
		if self.topic_to_show == "log":
			sys.stdout.write(msg.data)
			sys.stdout.flush()

	def loop(self):

		# loop
		while not rospy.core.is_shutdown():

			# state
			time.sleep(0.1)

	def __init__(self, topic_to_show):

		# state
		self.wait = False
		self.topic_to_show = topic_to_show

		# robot name
		topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

		# subscribe
		topic = topic_base_name + "/sensors/package"
		print ("subscribe", topic)
		self.sub_sensors = rospy.Subscriber(topic, miro.msg.sensors_package, self.callback_sensors, queue_size=5, tcp_nodelay=True)
		
		# subscribe
		topic = topic_base_name + "/sensors/log"
		print ("subscribe", topic)
		self.sub_log = rospy.Subscriber(topic, std_msgs.msg.String, self.callback_log, queue_size=5, tcp_nodelay=True)
		
		# subscribe
		topic = topic_base_name + "/sensors/mics"
		print ("subscribe", topic)
		self.sub_mics = rospy.Subscriber(topic, std_msgs.msg.Int16MultiArray, self.callback_mics, queue_size=5, tcp_nodelay=True)
		
		# subscribe
		topic = topic_base_name + "/control/cmd_vel"
		print ("subscribe", topic)
		self.sub_cmd_vel = rospy.Subscriber(topic, geometry_msgs.msg.TwistStamped, self.callback_cmd_vel, queue_size=5, tcp_nodelay=True)
		
if __name__ == "__main__":

	rospy.init_node("client_show_cliff", anonymous=True)
	main = client(sys.argv[1])
	main.loop()




