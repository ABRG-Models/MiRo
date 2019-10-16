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
import geometry_msgs

import math
import numpy as np
import sys, os, time
import curses

import miro2 as miro
 
################################################################

def error(msg):

	print(msg)
	sys.exit(0)

################################################################

class controller:

	def loop(self, stdscr):
	
		# turn off delay
		stdscr.nodelay(1)

		# output
		msg_cmd_vel = geometry_msgs.msg.TwistStamped()
		timeout = 0
		timeout_init = 6
		dtheta = 1.0
		dy = 0.2
		
		# instructions
		print "Use arrow keys to control MIRO\r\n"

		# loop
		while self.active and not rospy.core.is_shutdown():
		
			# this is a terrible way of getting the keydown
			# state, but it seems that it's a challenge to
			# do it well in Python for some reason.
			c = stdscr.getch()

			if c != -1:
				
				if c == 260:
					msg_cmd_vel.twist.linear.x = 0
					msg_cmd_vel.twist.angular.z = dtheta
					timeout = timeout_init
				elif c == 261:
					msg_cmd_vel.twist.linear.x = 0
					msg_cmd_vel.twist.angular.z = -dtheta
					timeout = timeout_init
				elif c == 259:
					msg_cmd_vel.twist.linear.x = dy
					msg_cmd_vel.twist.angular.z = 0
					timeout = timeout_init
				elif c == 258:
					msg_cmd_vel.twist.linear.x = -dy
					msg_cmd_vel.twist.angular.z = 0
					timeout = timeout_init
				elif c == 32:
					msg_cmd_vel.twist.linear.x = 0
					msg_cmd_vel.twist.angular.z = 0
					timeout = timeout_init
				else:
					#print c
					pass
			
			# flush buffer
			while c != -1:
				c = stdscr.getch()
			
			# yield
			time.sleep(0.1)
			self.t_now += 0.1
			
			# timeout
			if timeout > 0:
				timeout -= 1
				if timeout == 0:
					msg_cmd_vel.twist.linear.x = 0
					msg_cmd_vel.twist.angular.z = 0
				self.pub_cmd_vel.publish(msg_cmd_vel)
			
		# finalise report
		print "\n\nexit..."
		print " "

	def __init__(self, args):

		rospy.init_node("client_template", anonymous=True)

		# state
		self.active = False
		self.t_now = 0.0

		# robot name
		topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

		# publish
		topic = topic_base_name + "/control/cmd_vel"
		print ("publish", topic)
		self.pub_cmd_vel = rospy.Publisher(topic, geometry_msgs.msg.TwistStamped, queue_size=0)

		# report
		print """
Press arrow keys to control MIRO directly.
It's not something I'm proud of, but it works!"""

		# wait for connect
		print "\nwait for connect... ",
		sys.stdout.flush()
		time.sleep(1)
		print "OK"
		
		# set to active
		self.active = True

if __name__ == "__main__":

	# normal singular invocation
	main = controller(sys.argv[1:])
	curses.wrapper(main.loop)




