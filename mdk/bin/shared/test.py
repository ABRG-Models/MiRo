#!/usr/bin/python

import time
import gi
import os

gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GObject, GdkPixbuf, Gdk

import rospy
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, Int16MultiArray, String
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, BatteryState, Imu, Range, CompressedImage

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import math
import miro2 as miro



from matplotlib.backends.backend_gtk3agg import FigureCanvasGTK3Agg as FigureCanvas
import matplotlib.pyplot as plt
import matplotlib.animation as animation


rospy.init_node("test", anonymous=True)


current_robot = 'rob01'

# robot name
topic_base = "/" + os.getenv("MIRO_ROBOT_NAME") + "/"

pub_cmd_vel = rospy.Publisher(topic_base + "control/cmd_vel", TwistStamped, queue_size=0)


pub = rospy.Publisher('/miro/' + current_robot + '/platform/control', platform_control, queue_size=10)
image_pub = rospy.Publisher('/blockly/imageVariables/threshold/compressed', CompressedImage, queue_size=1, latch=True, tcp_nodelay=True)

rate = rospy.Rate(10)

 	
#-----------------------------END SETUP_MIRO---------------------------------

direction = "forward"
velocity = 2000
duration = 4

#-----------------------------START MOVE_DISTANCE---------------------------------
velocity = TwistStamped()


velocity = abs(velocity)
if direction != "forward":
    velocity = -velocity

if duration > 0:
    cycles=duration/rate.sleep_dur.to_sec()

    body_vel = Twist()
    body_vel.angular.x = 0
    body_vel.linear.x = velocity
    pub_cmd_vel.body_vel = body_vel

    #ensures that at least one node is connected before sending message
    while(pub.get_num_connections() == 0):
        rate.sleep()

    for i in range(int(cycles)):
        pub_cmd_vel.publish(velocity)
        rate.sleep()

    body_vel.linear.x = 0
    q.body_vel = body_vel
    pub.publish(pub_cmd_vel)	#Allow time for the move to be executed
#-----------------------------END MOVE_DISTANCE---------------------------------

