#!/usr/bin/python

import rospy
import time
import sys
import os#
import datetime
import numpy as np

import miro2 as miro

import std_msgs
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, BatteryState, Imu, Range, CompressedImage, Image

#Open CV
import cv2
from cv_bridge import CvBridge, CvBridgeError

################################################################

def error(msg):

    print(msg)
    sys.exit(0)

################################################################

class client_mics:

    def callback_pose(self, msg):

        self.pose = msg
        # print "received", mag

    def cam_left_callback(self, ros_image):
        try:
            self.cam_left_image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "bgr8")
            im_h, im_w = self.cam_left_image.shape[:2]
            if self.frame_w != im_w and self.frame_h != im_h:
                self.frame_w, self.frame_h = im_w, im_h
                self.cam_model.set_frame_size(self.frame_w, self.frame_h)
        except CvBridgeError as e:
            print("Conversion of left image failed \n")
            print(e)

    def cam_right_callback(self, ros_image):
        try:
            self.cam_right_image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "bgr8")
            im_h, im_w = self.cam_right_image.shape[:2]
            if self.frame_w != im_w and self.frame_h != im_h:
                self.frame_w, self.frame_h = im_w, im_h
                self.cam_model.set_frame_size(self.frame_w, self.frame_h)
        except CvBridgeError as e:
            print("Conversion of right image failed \n")
            print(e)



    def loop(self):

        # loop
        while not rospy.core.is_shutdown():

            something = self.find_ball("#0000FF",0,0)

            print "???", something

            time.sleep(0.01)



    def __init__(self):

        # config
        self.pose = []

        # Arrays to hold image topics
        self.cam_left_image = None
        self.cam_right_image = None

        # Create object to convert ROS images to OpenCV format
        self.image_converter = CvBridge()

        # robot name
        topic_base = "/" + os.getenv("MIRO_ROBOT_NAME") + "/"

        self.velocity = TwistStamped()

        self.pub_cmd_vel = rospy.Publisher(topic_base + "control/cmd_vel", TwistStamped, queue_size=0)

        self.debug_image_pub = rospy.Publisher("/miro/blockly_circles", Image, queue_size=0)

        # subscribe
        print ("subscribe", topic_base + "sensors/body_pose")
        self.sub_mics = rospy.Subscriber(topic_base + "sensors/body_pose", Pose2D, self.callback_pose)
        # Subscribe to Camera topics
        self.cam_left_sub = rospy.Subscriber(topic_base + "sensors/caml/compressed", CompressedImage,self.cam_left_callback)
        self.cam_right_sub = rospy.Subscriber(topic_base + "sensors/camr/compressed", CompressedImage,self.cam_right_callback)


        self.cam_model = miro.utils.CameraModel()

    # def pause(self):
    #     # Time at start of pause
    #     pause_start = time.time()
    #
    #     # Check for Pause Flag
    #     while (self.m_ready or self.wagging):
    #         # Check for presence of pause command
    #         if os.path.exists(FILE_WEB_CMD):
    #             with open(FILE_WEB_CMD, "r", os.O_NONBLOCK) as file:
    #                 self.web_cmd = file.read()
    #                 file.close()
    #         else:
    #             self.web_cmd = ""
    #
    #         if self.web_cmd == "pause":
    #             self.pause_flag = True
    #         else:
    #             break
    #
    #         time.sleep(0.1)
    #
    #     # Calculate pause time
    #     if self.pause_flag:
    #         pause_time = time.time() - pause_start
    #         self.timer_end_time = self.timer_end_time + pause_time
    #         self.pause_flag = False

    def find_ball(self, colour_str, cam_id, prop):
        # self.pause()
        if colour_str[0] != "#" and len(colour_str) != 7:
            print("colour choice should be a string in the form \"#RRGGBB\"")
            return
        if cam_id < 0 or cam_id > 1:
            return
        if prop < 0 or prop > 2:
            return

        # create colour code from user selected colour
        red = int(colour_str[1:3], 16)
        green = int(colour_str[3:5], 16)
        blue = int(colour_str[5:7], 16)
        bgr_colour = np.uint8([[[blue, green, red]]])
        hsv_colour = cv2.cvtColor(bgr_colour, cv2.COLOR_BGR2HSV)

        # extract boundaries for masking image
        target_hue = hsv_colour[0, 0][0]
        lower_bound = np.array([target_hue - 20, 70, 70])
        upper_bound = np.array([target_hue + 20, 255, 255])

        if np.shape(self.cam_left_image) != () and np.shape(self.cam_right_image) != ():
            # convert camera image to HSV colour space
            if cam_id == miro.constants.CAM_L:
                hsv_image = cv2.cvtColor(self.cam_left_image, cv2.COLOR_BGR2HSV)
                output = self.cam_left_image.copy()
            else:
                hsv_image = cv2.cvtColor(self.cam_right_image, cv2.COLOR_BGR2HSV)
                output = self.cam_right_image.copy()
        else:
            return None

        im_h = np.size(hsv_image, 0)
        im_w = np.size(hsv_image, 1)
        im_centre_h = im_h / 2.0
        im_centre_w = im_w / 2.0
        cv2.line(output, (0, int(round(im_centre_h))), (im_w, int(round(im_centre_h))), (100, 100, 100), 1)
        cv2.line(output, (int(round(im_centre_w)), 0), (int(round(im_centre_w)), im_h), (100, 100, 100), 1)

        # mask image
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        seg = mask

        # Do some processing
        seg = cv2.GaussianBlur(seg, (11, 11), 0)
        seg = cv2.erode(seg, None, iterations=2)
        seg = cv2.dilate(seg, None, iterations=2)

        # get circles
        circles = cv2.HoughCircles(seg, cv2.HOUGH_GRADIENT, 1, 40, param1=10, param2=20, minRadius=0, maxRadius=0)

        # Get largest circle
        max_circle = None
        max_circle_norm = [None, None, None]
        if circles is not None:
            self.max_rad = 0
            circles = np.uint16(np.around(circles))

            for c in circles[0, :]:
                cv2.circle(seg, (c[0], c[1]), c[2], (0, 255, 0), 2)

                if c[2] > self.max_rad:
                    self.max_rad = c[2]
                    max_circle = c
                    max_circle_norm[0] = int(round(((max_circle[0] - im_centre_w) / im_centre_w) * 100.0))
                    max_circle_norm[1] = int(round(-((max_circle[1] - im_centre_h) / im_centre_h) * 100.0))
                    max_circle_norm[2] = int(round((max_circle[2] / im_centre_w) * 100.0))

                # Debug Only
                cv2.circle(output, (max_circle[0], max_circle[1]), max_circle[2], (0, 255, 0), 2)
                cv2.circle(output, (max_circle[0], max_circle[1]), 1, (0, 255, 0), 2)
                location_str = "x: " + str(max_circle_norm[0]) + "," + "y: " + str(
                    max_circle_norm[1]) + "," + "r: " + str(max_circle[2])
                text_y_offset = 18
                for i, line in enumerate(location_str.split(",")):
                    text_y = max_circle[1] - text_y_offset + i * text_y_offset
                    cv2.putText(output, line, (max_circle[0] + 5, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
                    cv2.putText(output, line, (max_circle[0] + 5, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0),
                                1)

        else:
            return None

        debug_image = self.image_converter.cv2_to_imgmsg(output, "bgr8")
        self.debug_image_pub.publish(debug_image)
        return max_circle_norm[prop]

    def get_ball_loc(self, colour_str):
        # self.pause()
        if colour_str[0] != "#" and len(colour_str) != 7:
            print("colour choice should be a string in the form \"#RRGGBB\"")
            return

        if np.shape(self.cam_left_image) == () or np.shape(self.cam_right_image) == ():
            return

        # create colour code from user selected colour
        red = int(colour_str[1:3], 16)
        green = int(colour_str[3:5], 16)
        blue = int(colour_str[5:7], 16)
        bgr_colour = np.uint8([[[blue, green, red]]])
        hsv_colour = cv2.cvtColor(bgr_colour, cv2.COLOR_BGR2HSV)

        # extract boundaries for masking image
        target_hue = hsv_colour[0, 0][0]
        lower_bound = np.array([target_hue - 20, 70, 70])
        upper_bound = np.array([target_hue + 20, 255, 255])

        # Search for largest circle in either frame
        largest_circle = None
        circle_loc = None
        circle_cam = None
        largest_radius = 0
        for camera in range(0, 2):

            if camera == miro.constants.CAM_L:
                hsv_image = cv2.cvtColor(self.cam_left_image, cv2.COLOR_BGR2HSV)
            else:
                hsv_image = cv2.cvtColor(self.cam_right_image, cv2.COLOR_BGR2HSV)

            # mask image
            mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
            seg = mask

            # Do some processing
            seg = cv2.GaussianBlur(seg, (11, 11), 0)
            seg = cv2.erode(seg, None, iterations=2)
            seg = cv2.dilate(seg, None, iterations=2)

            # get circles
            circles = cv2.HoughCircles(seg, cv2.HOUGH_GRADIENT, 1, 40, param1=10, param2=33, minRadius=0, maxRadius=0)

            # Search through circles for largest
            if circles is not None:
                circles = np.uint16(np.around(circles))
                for circle in circles[0, :]:
                    if circle[2] > largest_radius:
                        largest_radius = circle[2]
                        largest_circle = circle
                        circle_cam = camera
            else:
                pass
        circle_loc = [largest_circle[0], largest_circle[1]]
        view_line = self.cam_model.p2v(circle_loc)
        circle_point = self.cam_model.v2oh(circle_cam, view_line, 1.0)
        return circle_point


if __name__ == "__main__":

    rospy.init_node("client_mics", anonymous=True)
    main = client_mics()
    main.loop()




