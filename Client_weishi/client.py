#!/usr/bin/python
#This is a MIRO ROS client for Python.

# client Library 提供ROS编程的库（接口）
import rospy
# geometry_msgs provides messages for common geometric primitives such as points, vectors, and poses.
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
from face_detection import *

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

        self.detected_faces = face_detection(image)



    def loop(self):
        # loop
        while self.active and not rospy.core.is_shutdown():
            if self.detected_faces != None:
                cv2.imshow('image', self.detected_faces)
                # https://www.jianshu.com/p/30c40d7ce5dc
                cv2.waitKey(1)

            if self.roi_color != None:
                cv2.imshow('face', self.roi_color)
                cv2.waitKey(1)

            # yield
            time.sleep(0.01)
            self.t_now = self.t_now + 0.01

        cv2.destroyAllWindows()


# args 参数列表
    def __init__(self, args):
        # 注册和初始化node
        rospy.init_node("client", anonymous=True)

        # state
        self.t_now = 0.0
        self.active = False

        # inputs
        self.package = None
        self.detected_faces = None
        self.roi_color = None

        # handle args
        for arg in args:
            # 在参数中找 = 并返回索引 如不含则返回-1
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
        # 定义topic --- 字符串
        topic = topic_base_name + "/control/cmd_vel"
        print("publish", topic)
        # 返回publisher的对象，pub_cmd_vel是一个publisher,还没有真正的发布消息
        # 同步 发收双方具有同频同相的同步时钟信号 在同步时钟的控制下逐位发送/接收
        self.pub_cmd_vel = rospy.Publisher(topic, TwistStamped, queue_size=0)

        # subscribe
        # sensors/caml, Frames from the left eye camera (sample rate is variable, see control/command).
        topicCamLeft = topic_base_name + "/sensors/caml/compressed"
        print("subscribe", topicCamLeft)
        # 异步 异步通信在发送字符时，发送端可以在任意时刻开始发送字符 接收端必须时刻做好接收的准备
        self.sub_caml = rospy.Subscriber(topicCamLeft, CompressedImage, self.callback_caml, queue_size=1,
                                         tcp_nodelay=True)


        # wait for connect
        print
        "wait for connect..."
        # 函数推迟调用线程的运行，可通过参数t指秒数，表示进程挂起的时间。
        time.sleep(1)

        # set to active
        self.active = True





if __name__ == "__main__":
    main = controller(sys.argv[1:])
    main.loop()




