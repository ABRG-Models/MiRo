#!/usr/bin/python
#This is a MIRO ROS client for Python.

import rospy
# http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html
from geometry_msgs.msg import TwistStamped

import math
import numpy as np
import time
import sys
import os

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from detect_primary_user import *

# The miro2 Python module provides constants and tools for working with MiRo from Python.
# with PYTHONPATH=~/mdk/share/python:$PYTHONPATH
import miro2 as miro


################################################################

def error(msg):
    print(msg)
    sys.exit(0)

################################################################

class controller:
    def callback_caml(self, ros_image):
        # ignore until active
        if not self.active:
            return

        self.image_converter = CvBridge()
        # convert compressed ROS image to raw CV image
        image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "bgr8")

        # detect face and return the "face"
        self.detected_faces, self.roi_color = face_detection(image)
        


    def loop(self):
        # loop
        while self.active and not rospy.core.is_shutdown():
            if self.detected_faces != None:
                cv2.imshow('image', self.detected_faces)
                cv2.waitKey(1)
                #face recognition
                #self.is_primary = face_recognition(self.detected_faces)

            # detect the face
            if self.roi_color != None:
                save_face(self.roi_color)
                cv2.imshow('face', self.roi_color)
                face_recognition(self.roi_color)
                cv2.waitKey(1)
		
            # yield
            time.sleep(0.01)
            self.t_now = self.t_now + 0.01
            
            # face recognition  run once/sec
            #time.sleep(1000)

        cv2.destroyAllWindows()


    def __init__(self, args):
        rospy.init_node("client", anonymous=True)
       # init_rekognition()

        # state
        self.t_now = 0.0
        self.active = False

        # inputs
        self.package = None
        self.detected_faces = None
        self.roi_color = None

        # handle args
        for arg in args:
            f = arg.find('=')
            if f == -1:
                key = arg
                val = ""
            else:
                key = arg[:f]
                val = arg[f + 1:]
            if key == "pass":
                pass
            else:
                error("argument not recognised \"" + arg + "\"")

        # robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # publish
       # topic = topic_base_name + "/control/cmd_vel"
       # print("publish", topic)
       # self.pub_cmd_vel = rospy.Publisher(topic, TwistStamped, queue_size=0)

        # subscribe
        # sensors/caml, Frames from the left eye camera (sample rate is variable, see control/command).
        topicCamLeft = topic_base_name + "/sensors/caml/compressed"
        print("subscribe", topicCamLeft)
        self.sub_caml = rospy.Subscriber(topicCamLeft, CompressedImage, self.callback_caml, queue_size=1,
                                         tcp_nodelay=True)


        # wait for connect
        print "wait for connect..."
        time.sleep(1)

        # set to active
        self.active = True

if __name__ == "__main__":
    main = controller(sys.argv[1:])
    main.loop()




