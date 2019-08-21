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
import datetime
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

    def track(self):

        st_track = time.time()

        epsilon = 10.0
        tilt, lift, yaw, pitch = range(4)


        hf = self.y_primary - self.image_height/2
        df = self.x_primary - self.image_width/2

        #=====================test===============================
        # print('============coordinate of face center=================')
        # print('df: ', df, 'hf: ', hf)
        #========================================================

        # Horizon
        if df > epsilon and self.kin_joints.position[yaw] > miro.constants.YAW_RAD_MIN:
            # Move head to the right
            # print('df =  ', df, '  move head to the right')
            # b = self.kin_joints.position[yaw]
            self.kin_joints.position[yaw] = self.kin_joints.position[yaw] + (0.7*df*miro.constants.YAW_RAD_MIN/self.image_width)
            # print('after rad: ', (self.kin_joints.position[yaw]-b)/0.0174532)

        elif df < -epsilon and self.kin_joints.position[yaw] < miro.constants.YAW_RAD_MAX:
            # Move head to the left
            # print('df < epsilon. move head to the left')
            # b = self.kin_joints.position[yaw]
            self.kin_joints.position[yaw] = self.kin_joints.position[yaw] - (0.7*df*miro.constants.YAW_RAD_MAX/self.image_width)
            # print('after rad: ', (self.kin_joints.position[yaw]-b)/0.0174532)

        # Vertical

        if hf < -epsilon and self.kin_joints.position[lift] > miro.constants.LIFT_RAD_MIN:
            # Move head up
            # print('hf > epsilon. move head up')
            # a = self.kin_joints.position[lift]
            self.kin_joints.position[lift] = self.kin_joints.position[lift] + hf*miro.constants.LIFT_RAD_MIN/self.image_height
            # print('after moving, degree: ', (self.kin_joints.position[lift]-a)/0.0174532)

        elif hf > epsilon and self.kin_joints.position[lift] < miro.constants.LIFT_RAD_MAX:
            # Move head down
            print('hf < epsilon. move head down')
            a = self.kin_joints.position[lift]
            self.kin_joints.position[lift] = self.kin_joints.position[lift] + hf*miro.constants.LIFT_RAD_MAX/self.image_height
            print('after moving, degree: ', self.kin_joints.position[lift]-a)

        self.pub_kin.publish(self.kin_joints)

        end_track = time.time()
        print('******************time of tracking: ', end_track - st_track)

        time.sleep(0.3)

    def do_detection(self):

        self.image_converter = CvBridge()
        # convert compressed ROS image to raw CV image
        self.image = self.image_converter.compressed_imgmsg_to_cv2(self.image, "bgr8")

        # detect face and return the "face"
        # st = time.time()
        self.detected_faces, self.faces = self.det_pri_user.face_detection(self.image)
        # et = time.time()
        # print('******************time of detection: ', et - st)

        self.image = None

    def do_recognition(self):

        # PRIMARY USER RECOGNITION, GET X,Y OF THE PRIMARY USER
        st = time.time()
        # self.faces is the dictionary of faces include roi & coordinate of face center
        self.primary_detected, self.face_user = self.det_pri_user.face_recognition(self.faces)
        et = time.time()
        print('******************time of recognition: ', et - st)

        # x = self.face_user[0]
        # y = self.face_user[1]
        # w = self.face_user[2]
        # h = self.face_user[3]
        if self.face_user != []:
            self.x_primary = self.face_user[0] + self.face_user[2]/2
            self.y_primary = self.face_user[1] + self.face_user[3]/2

    def callback_caml(self, ros_image):
        # ignore until active
        if not self.active:
            return

        self.image = ros_image

    def reset(self):
        self.kin_joints.position = [0.0, math.radians(34.0), 0.0, 0.0]
        self.pub_kin.publish(self.kin_joints)
        time.sleep(0.1)

    def get_face_distance(self, wf, hf):
        d = 0.0
        area_face = wf * hf
        area_image = 640 * 360
        # print ('&&&&&  face width  &&&&&    ', wf)
        # print ('&&&&&  face height &&&&&    ', hf)
        d = (area_face * 1.0) / area_image

        return d

    def move(self, wheel_speed):

        # Convert to command velocity
        (dr, dtheta) = miro.utils.wheel_speed2cmd_vel(wheel_speed)

        # Set velocity values
        self.velocity.twist.linear.x = dr
        self.velocity.twist.angular.z = dtheta

        self.pub_cmd_vel.publish(self.velocity)

    def miro_approach(self):
        tilt, lift, yaw, pitch = range(4)
        if self.kin_joints.position[yaw] < -(20*0.0174532):
            # move right
            print('@@@@@@', self.kin_joints.position[yaw]/0.0174532)
            print('@@@@@@move right')
            self.run_sec('R')
        elif self.kin_joints.position[yaw] > (20*0.0174532):
            # move left
            print('@@@@@@', self.kin_joints.position[yaw]/0.0174532)
            print('@@@@@@move left')
            self.run_sec('L')
        else:
            # move forward
            print('@@@@@@move forward')
            self.run_sec('U')
        time.sleep(2)


        print('@@@@@@after running for 0.2s')

        # wheel_speed = [1.0, 1.0]
        #
        # self.do_detection()
        # if self.faces != {}:
        #     self.do_recognition()
        #     if self.primary_detected == True:
        #         # get close to user
        #         d = self.get_face_distance(self.face_user[2], self.face_user[3])
        #         while d > 0.05:
        #             print('loop distance: ', d)
        #             self.move(wheel_speed)
        #             self.do_detection()
        #             if self.faces != {}:
        #                 self.do_recognition()
        #                 if self.primary_detected == True:
        #                     # get close to user
        #                     d = self.get_face_distance(self.face_user[2], self.face_user[3])
        # time.sleep(1)

    # miro go straight for 0.1s
    def run_sec(self, direction):

        if direction == 'R':
            wheel_speed = [0.2, 0.0]
        elif direction == 'L':
            wheel_speed = [0.0, 0.2]
        elif direction == 'U':
            wheel_speed = [0.2, 0.2]

        start = datetime.datetime.now()

        while True:
            end = datetime.datetime.now()

            if(end - start).microseconds > 200000:
                wheel_speed0 = [0.0, 0.0]
                self.move(wheel_speed0)
                break
            else:
                self.move(wheel_speed)



    def loop(self):
        self.reset()
        # loop
        while self.active and not rospy.core.is_shutdown():
            # face detection
            # 'Add new condition, do the recognition once of 100 frames'
            if self.image != None:
                self.do_detection()

            # track primary user
            if self.faces != {}:
                self.do_recognition()
                if self.primary_detected == True:
                    self.track()
                    # get close to user
                    distance = self.get_face_distance(self.face_user[2], self.face_user[3])
                    print('&&&&&  distance  &&&&&    ', distance)
                    if distance < 0.02:
                        self.miro_approach()

            # track any face
            # if self.x_primary != None:
            #     self.track(self.x_primary, self.y_primary, 640, 360)

            # Show the frames and ROI
            if self.detected_faces != None:
                cv2.imshow('detected_face', self.detected_faces)
                cv2.waitKey(1)

            # if self.roi_color != None:
            #     cv2.imshow('face', self.roi_color)
            #     cv2.waitKey(1)

                
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
        # self.roi_color = None
        self.faces = {}
        self.face_user = []
        self.x_primary = None
        self.y_primary = None
        self.primary_detected = False
        self.image_width = 640
        self.image_height = 360

        self.velocity = TwistStamped()

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
        topic = topic_base_name + "/control/cmd_vel"
        print("publish", topic)
        self.pub_cmd_vel = rospy.Publisher(topic, TwistStamped, queue_size=0)

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




