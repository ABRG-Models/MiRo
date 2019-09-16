#!/usr/bin/python
# This is a MIRO ROS client for Python.

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
import matplotlib.pyplot as plt
import numpy.matlib

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

        # image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "bgr8")
        # if self.i < 20:
        #     cv2.imwrite(str(self.i) + '.jpg', image)
        #     time.sleep(2)

        self.i += 1


    def loop(self):
        pass
        # loop
        while self.active and not rospy.core.is_shutdown():
            if self.i == 20:
                print('i = 20')
                for i in range(20):
                    print('processing' + str(i))
                    image = self.getImage(str(i)+'.jpg')
                    self.process(image)

                plt.show()


            # if self.i == 1:
            #     print('is 1 ')
            #     self.reader()
            #     plt.show()









        # cv2.destroyAllWindows()

    def getImage(self,name):
        img = cv2.imread(name)
        res = cv2.resize(img, dsize = (self.s,self.s), interpolation=cv2.INTER_CUBIC)
        res = np.mean(res,2)


        return res


    def process(self,res):

        self.X[self.counter, :] = res.flatten()
        # repeat this n times
        # Apply PCA
        h, w = self.X.shape
        mu = np.reshape(np.mean(self.X, 1), (h, 1))
        # Substract mean
        self.X = self.X - np.matlib.repmat(mu, 1, w)
        np.savetxt('test.out', self.X, delimiter=',')

        # Covariance matrix
        C = np.dot(self.X.T, self.X)
        # Eigendecpmposition
        d, v = np.linalg.eig(C)
        # This the projection matrix
        v = v[:, 0:2]
        # Compute the projection
        z = np.dot(v.T, res.flatten())
        plt.plot(z[0], z[1], 'r*')
        self.counter += 1



    def reader(self):
        print('a')
        self.X = np.loadtxt('test.out', delimiter=',')
        print('b')


        # Covariance matrix
        C = np.dot(self.X.T, self.X)
        print(self.X)
        print('c')
        # Eigendecpmposition
        d, v = np.linalg.eig(C)
        print('d')
        # This the projection matrix
        v = v[:, 0:2]
        # Compute the projection
        print('e')
        for i in self.X:
            print('in')
            z = np.dot(v.T, i)
            plt.plot(z[0], z[1], 'r*')


    def __init__(self, args):
        rospy.init_node("client", anonymous=True)

        self.n = 20
        self.s = 64
        self.X = np.zeros((self.n,self.s*self.s))

       # sim
        self.t = time.time()
        self.i = 0
        self.counter = 0


        # state
        self.t_now = 0.0
        self.active = False

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
        print
        "wait for connect..."
        time.sleep(1)

        # set to active
        self.active = True


if __name__ == "__main__":
    main = controller(sys.argv[1:])
    main.loop()



