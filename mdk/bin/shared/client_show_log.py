#!/usr/bin/python

import rospy
import std_msgs
import time
import sys
import os

################################################################

def error(msg):

	print(msg)
	sys.exit(0)

################################################################

class client:

	def callback_log(self, msg):

		sys.stdout.write(msg.data)
		sys.stdout.flush()

	def loop(self):

		# loop
		while not rospy.core.is_shutdown():

			# state
			time.sleep(0.1)

	def __init__(self):

		# robot name
		topic_base = "/" + os.getenv("MIRO_ROBOT_NAME") + "/"

		# subscribe
		topic = topic_base + "sensors/log"
		print ("subscribe", topic)
		self.sub_log = rospy.Subscriber(topic,
				std_msgs.msg.String, self.callback_log)

if __name__ == "__main__":

	rospy.init_node("client_show_log", anonymous=True)
	main = client()
	main.loop()




