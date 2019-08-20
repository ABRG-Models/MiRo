#!/usr/bin/python

import rospy
import time
import sys
import os
import numpy as np
import math

import std_msgs.msg as Float32MultiArray
import miro2 as miro
from sensor_msgs.msg import JointState


droop, wag, left_eye, right_eye, left_ear, right_ear = range(6)
tilt, lift, yaw, pitch = range(4)

################################################################

def error(msg):

    print(msg)
    sys.exit(0)

################################################################

class client_lift:

    def callback_package(self, msg):

        x = msg.sonar.range

        # print "sonar", x

    def loop(self):

        # loop
        while not rospy.core.is_shutdown():

            # sleep
            time.sleep(0.01)

    def __init__(self):

        # config
        self.kin_joints = JointState()
        self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
        self.kin_joints.position = [0.0, math.radians(57.0), 0.0, 0.0]
        self.cos_joints = Float32MultiArray()
        self.cos_joints.data = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]
        self.on_ResetCosButton_clicked()

        # robot name
        topic_base = "/" + os.getenv("MIRO_ROBOT_NAME") + "/"

        self.pub_kin = rospy.Publisher(topic_base + "control/kinematic_joints", JointState, queue_size=0)
        self.pub_cos = rospy.Publisher(topic_base + "control/cosmetic_joints", Float32MultiArray, queue_size=0)

        # subscribe
        topic = topic_base + "sensors/package"
        print ("subscribe", topic)
        self.sub_package = rospy.Subscriber(topic, miro.msg.sensors_package, self.callback_package)

        # self.pub_kin.publish(self.kin_joints)

    def on_ResetCosButton_clicked(self, *args):

        # self.WagRateControl.set_value(31.0)

        self.cos_joints.data[droop] = miro.constants.DROOP_CALIB
        self.cos_joints.data[wag] = miro.constants.WAG_CALIB
        self.cos_joints.data[left_eye] = miro.constants.EYE_CALIB
        self.cos_joints.data[right_eye] = miro.constants.EYE_CALIB
        self.cos_joints.data[left_ear] = miro.constants.EAR_CALIB
        self.cos_joints.data[right_ear] = miro.constants.EAR_CALIB
if __name__ == "__main__":

    rospy.init_node("client_lift", anonymous=True)
    main = client_lift()
    main.loop()




