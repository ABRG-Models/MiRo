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
from sensor_msgs.msg import CompressedImage, JointState
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

    def callback_package(self, msg):

        # ignore until active
        if not self.active:
            return

        # store
        self.sensors = msg

    def track( self, x_face, y_face, width, height ):

        epsilon = 10.0
        tilt, lift, yaw, pitch = range(4)
        hf = y_face - height/2
        df = x_face - width/2

        #=====================test===============================
        print('============coordinate of face center=================')
        print('face_x: ', x_face, 'face_y: ' , y_face)
        print('df: ', df, 'hf: ', hf)
        print()
        #========================================================

        # Horizon
        if df > epsilon and self.kin_joints.position[yaw] > miro.constants.YAW_RAD_MIN:
            # Move head to the right
            print('df > epsilon. move head to the right')
            print('before rad: ', self.kin_joints.position[yaw])
            self.kin_joints.position[yaw] = self.kin_joints.position[yaw] + (df*miro.constants.YAW_RAD_MIN/width)
            print('after rad: ', self.kin_joints.position[yaw])

        elif df < -epsilon  and self.kin_joints.position[yaw] <  miro.constants.YAW_RAD_MAX:
            # Move head to the left
            print('df < epsilon. move head to the left')
            print('before rad: ', self.kin_joints.position[yaw])
            self.kin_joints.position[yaw] = self.kin_joints.position[yaw] - (df*miro.constants.YAW_RAD_MAX/width)
            print('after rad: ', self.kin_joints.position[yaw])

        # Vertical
        # if hf > epsilon and self.kin_joints.position[lift] > miro.constants.LIFT_RAD_MIN:
        #     # Move head up
        #     self.kin_joints.position[lift] = self.kin_joints.position[lift] + math.radians(10.0)
        #
        # elif hf < -epsilon and self.kin_joints.position[lift] > miro.constants.LIFT_RAD_MIN:
        #     # Move head down
        #     self.kin_joints.position[lift] = self.kin_joints.position[lift] - math.radians(10.0)

        self.pub_kin.publish(self.kin_joints)
        self.primary_detected = False
        time.sleep(3)

    def do_recognition(self):
        # detect face and return the "face"
        st = time.time()
        print('The current time: ', st)
        self.detected_faces, self.roi_color, self.x_primary, self.y_primary = self.det_pri_user.face_detection(self.image)
        et = time.time()
        print('time of detection: ', et - st)

       # if self.roi_color != None:
            # save ROI
           # self.det_pri_user.save_face(self.roi_color)

            # PRIMARY USER RECOGNITION, GET X,Y OF THE PRIMARY USER
           # st = time.time()
           # self.primary_detected, self.x_primary, self.y_primary, width, height = self.det_pri_user.face_recognition(
           #     self.roi_color)
           # et = time.time()
           # print('time of fr: ', et - st)

        self.image = None

    def callback_caml(self, ros_image):
        #print('callback here')
        #print(self.i, 'th callback_caml: ', time.time()-self.t)
        #self.i += 1

        # ignore until active
        if not self.active:
            return

        self.image_converter = CvBridge()
        # convert compressed ROS image to raw CV image
        self.image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "bgr8")
            
        if self.image != None:
            self.do_recognition()

        #if self.primary_detected == True:
            #self.track( self.x_primary, self.y_primary, 640, 360 )

        if self.x_primary != None:
            self.track( self.x_primary, self.y_primary, 640, 360 )



    def reset(self):
        self.kin_joints.position = [0.0, math.radians(34.0), 0.0, 0.0]
        self.pub_kin.publish(self.kin_joints)
        time.sleep(1)

    def loop(self):
        self.reset()
        # loop
        while self.active and not rospy.core.is_shutdown():
            if self.detected_faces != None:
               cv2.imshow('detected_face', self.detected_faces)
               cv2.waitKey(1)

            if self.roi_color != None:
                cv2.imshow('face', self.roi_color)
                cv2.waitKey(1)
                
            # yield
            time.sleep(0.01)
            self.t_now = self.t_now + 0.01
            
            # face recognition  run once/sec
            #time.sleep(1)

        cv2.destroyAllWindows()


    def __init__(self, args):
        rospy.init_node("client", anonymous=True)
        
        # sim
        self.t = time.time()
        self.i = 0
        self.j = 0

        # state
        self.t_now = 0.0
        self.active = False

        # inputs
        self.package = None
        self.detected_faces = None
        self.roi_color = None
        self.x_primary = None


        # the object of detect_primary_user
        self.det_pri_user = detect_primary_user()
        # create the diff images of user collection
        self.det_pri_user.face_collection()
        

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
        # subscribe
        topic = topic_base_name + "/sensors/package"
        print ("subscribe", topic)
        self.sub_package = rospy.Subscriber(topic, miro.msg.sensors_package, self.callback_package)

        # publish
        topic = topic_base_name + "/control/kinematic_joints"
        print ("publish", topic)
        self.pub_kin = rospy.Publisher(topic, JointState, queue_size=0)

        self.kin_joints = JointState()
        self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]

        # wait for connect
        print "wait for connect..."
        time.sleep(1)

        # set to active
        self.active = True
        self.primary_detected = False
        self.image = None

if __name__ == "__main__":
    main = controller(sys.argv[1:])
    main.loop()




