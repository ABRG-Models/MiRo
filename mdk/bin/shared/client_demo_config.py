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
import std_msgs

import math
import numpy as np
import time
import sys
import os
import curses

import miro2 as miro

################################################################

def error(msg):

	print(msg)
	sys.exit(0)

def usage():

	print ("""
Usage:
	client_demo_config.py demo_flags=...

Options:
	demo_flags
		Set the initial demo_flags value (e.g. "v").
	""")
	sys.exit(0)

def fmtflt(f):

	return "{:.3f}".format(f)

################################################################

class controller:

	def callback_config(self, msg):

		print "demo_flags", msg.data

	def loop(self, win):

		# state
		t_now = 0.0

		# message
		msg_config = std_msgs.msg.String()

		# configure
		win.nodelay(True)
		win.clear()
		win.addstr("Detected key:")

		# loop
		while self.active and not rospy.core.is_shutdown():

			key = None
			try:
				key = win.getkey()
			except:
				pass
			print "key pressed:", key
			win.addstr("\n")

			# send string
			win.clear()
			if key is None:
				msg_config.data = "ping"
			elif key in "vtrnscudMSFBThaHmPQ":
				msg_config.data = "f" + key
			elif key in "012345":
				msg_config.data = "p" + key
			else:
				print "(key press not understood)"
			self.pub_config.publish(msg_config)

			# state
			T = 1
			time.sleep(T)
			self.count = self.count + 1
			t_now = t_now + T

	def __init__(self, args):

		rospy.init_node("client_demo_config", anonymous=True)

		# state
		self.count = 0
		self.demo_flags = None

		# handle args
		for arg in args:
			f = arg.find('=')
			if f == -1:
				key = arg
				val = ""
			else:
				key = arg[:f]
				val = arg[f+1:]
			if key == "demo_flags":
				self.demo_flags = val
			else:
				error("argument not recognised \"" + arg + "\"")

		# robot name
		topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

		# publish
		topic = topic_base_name + "/core/config/command"
		print ("publish", topic)
		self.pub_config = rospy.Publisher(topic, std_msgs.msg.String, queue_size=0)

		# subscribe
		topic = topic_base_name + "/core/config/state"
		print ("subscribe", topic)
		self.sub_config = rospy.Subscriber(topic, std_msgs.msg.String, self.callback_config, queue_size=5, tcp_nodelay=True)

		# wait for connect
		print "wait for connect..."
		time.sleep(1)

		# set to active
		self.active = True

def execfunc(win):

	# normal singular invocation
	main = controller(sys.argv[1:])
	main.loop(win)

if __name__ == "__main__":

	# invoke curses
	curses.wrapper(execfunc)



