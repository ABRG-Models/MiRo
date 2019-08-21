#!/usr/bin/python

import rospy
import time
import sys
import os#
import datetime
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import math

import miro2 as miro

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, BatteryState, Imu, Range, CompressedImage, Image
from std_msgs.msg import UInt16MultiArray, Int16MultiArray

#Open CV
import cv2
from cv_bridge import CvBridge, CvBridgeError
import wave, struct
import Speech2Text as Speech2Text


# amount to keep the buffer stuffed - larger numbers mean
# less prone to dropout, but higher latency when we stop
# streaming. with a read-out rate of 8k, 4000 samples will
# buffer for half of a second, for instance.
BUFFER_STUFF_SAMPLES = 4000

# messages larger than this will be dropped by the receiver,
# however, so - whilst we can stuff the buffer more than this -
# we can only send this many samples in any single message.
MAX_STREAM_MSG_SIZE = (4096 - 48)

# using a margin avoids sending many small messages - instead
# we will send a smaller number of larger messages, at the cost
# of being less precise in respecting our buffer stuffing target.
BUFFER_MARGIN = 1000
BUFFER_MAX = BUFFER_STUFF_SAMPLES + BUFFER_MARGIN
BUFFER_MIN = BUFFER_STUFF_SAMPLES - BUFFER_MARGIN

# how long to record before playing back in seconds?
RECORD_TIME = 2

# microphone sample rate (also available at miro2.constants)
MIC_SAMPLE_RATE = 20000

# sample count
SAMPLE_COUNT = RECORD_TIME * MIC_SAMPLE_RATE
OUR_FILE_NAME = '/home/miro/mdk/bin/shared/output.wav'

droop, wag, left_eye, right_eye, left_ear, right_ear = range(6)
tilt, lift, yaw, pitch = range(4)
################################################################

def error(msg):

    print(msg)
    sys.exit(0)

################################################################

class QLearningTable:
    def __init__(self, action_space,learning_rate=0.2, discount=0.9, e_greedy=0.9, ka=0.4, kb=0.6, beta=0.4):
        self.lr = learning_rate
        self.gamma = discount
        self.epsilon = e_greedy
        self.actions = action_space

        self.ka = ka
        self.kb = kb
        self.beta = beta
        # self.num_states = np.array([5, 5])
        # self.num_actions = 7
        # self.Q_table = np.zeros((self.num_states[0], self.num_states[1], self.num_actions))

        self.q_table = pd.DataFrame(columns=self.actions, dtype=np.float64)
        self.e_table = pd.DataFrame(columns=self.actions, dtype=np.float64)

    def check_state_exist(self, state):
        if state not in self.q_table.index:
            # append new state to q table
            self.q_table = self.q_table.append(
                pd.Series(
                    [0]*len(self.actions),
                    index=self.q_table.columns,
                    name=state,
                )
            )
            # append new state to e table
            self.e_table = self.e_table.append(
                pd.Series(
                    [0] * len(self.actions),
                    index=self.e_table.columns,
                    name=state,
                )
            )

    def print_Tabel(self):
        print('Q',self.q_table)
        print('E',self.e_table)

    def choose_action(self, state):
        self.check_state_exist(state)

        # action selection
        if np.random.rand() < self.epsilon:
            # choose random action
            action = np.random.choice(self.actions)
        else:
            # choose best action
            state_action = (1 - self.beta) * self.q_table.loc[state, :] + self.beta * self.e_table.loc[state, :]
            # some actions may have the same value, randomly choose on in these actions
            action = np.random.choice(state_action[state_action == np.max(state_action)].index)
        return action

    def learn(self, state, action, reward, new_state, e_value):
        self.check_state_exist(new_state)
        q_predict = self.q_table.loc[state, action]
        if new_state != 'terminal':
            q_target = reward + self.gamma * self.q_table.loc[new_state, :].max()  # next state is not terminal
        else:
            q_target = reward  # next state is terminal
        self.q_table.loc[state, action] += self.lr * (q_target - q_predict)  # update

        # update e table
        if e_value != 0:
            self.e_table.loc[state, action] += e_value
            self.e_table.loc[state, :] *= self.kb
            self.e_table.loc[state, action] *= self.ka/self.kb


################################################################

class client_findball_shaping:
    def callback_package(self, msg):
        self.sonar = msg.sonar.range

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

    def callback_mics(self, msg):

        # if recording
        if not self.micbuf is None:

            # append mic data to store
            data = np.array(msg.data, 'int16')
            x = np.reshape(data, (-1, 500))
            self.micbuf = np.concatenate((self.micbuf, x.T))

            # report
            sys.stdout.write(".")
            sys.stdout.flush()

            # finished recording?
            if self.micbuf.shape[0] >= SAMPLE_COUNT:
                # end recording
                self.outbuf = self.micbuf
                self.micbuf = None
                print " OK!"

    def loop(self):
        # loop
        while not rospy.core.is_shutdown():
            self.kin_cos_init()
            f = lambda x, min_x: max(min_x, min(1.0, 1.0 - np.log10((x + 1) / 25.0)))

            reward_list = []

            for i in range(self.episode):
                print "i", i
                done = False
                state = self.get_state()
                tot_reward = 0
                step = 0
                e_value = 0

                while not done:
                    step += 1
                    action = self.Q.choose_action(str(state))
                    # Get next state and reward
                    state2, reward, done = self.step(action)

                    if not self.outbuf is None:
                        time.sleep(0.02)
                        self.record_audio()
                        e_value = self.analyse_speech()
                        if e_value is None:
                            e_value = 0

                    if step == 100:
                        done = True
                        state2 = 'terminal'

                    # Allow for terminal states
                    self.Q.learn(str(state), action, reward, str(state2), e_value)

                    state = state2
                    tot_reward += reward
                    print "steps", step
                    print "reward", reward
                    print "new state", state
                    time.sleep(0.01)

                reward_list.append(tot_reward)
                self.Q.epsilon = f(i, 0.0)
                self.Q.lr = f(i, 0.01)
                self.Q.beta = f(i,0.08)


            print self.Q.print_Tabel()
            plt.figure(1)
            plt.plot(reward_list)
            plt.show()
            self.save_QTable(self.Q.q_table)
            readQcsv = self.import_QTable()
            print "import q csv\n", readQcsv

            self.save_ETable(self.Q.e_table)
            readEcsv = self.import_ETable()
            print "import e csv\n", readEcsv

            break


    def __init__(self):

        # config
        self.micbuf = np.zeros((0, 4), 'uint16')
        self.outbuf = None
        # Arrays to hold image topics
        self.cam_left_image = None
        self.cam_right_image = None
        # Create object to convert ROS images to OpenCV format
        self.image_converter = CvBridge()

        self.sonar = 0.58

        self.velocity = TwistStamped()
        self.action_space = ['PUSH','STEP_BACK', 'TURN_L_45', 'TURN_L_90', 'TURN_R_45', 'TURN_R_90', 'TURN_180']
        self.goal = np.array([4, 2])
        self.episode = 500
        self.Q = QLearningTable(self.action_space)

        self.kin_joints = JointState()
        self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
        self.kin_joints.position = [0.0, math.radians(57.0), 0.0, 0.0]
        self.cos_joints = Float32MultiArray()
        self.cos_joints.data = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]
        self.cos_joints.data[droop] = miro.constants.DROOP_CALIB
        self.cos_joints.data[wag] = miro.constants.WAG_CALIB
        self.cos_joints.data[left_eye] = miro.constants.EYE_DEFAULT
        self.cos_joints.data[right_eye] = miro.constants.EYE_DEFAULT
        self.cos_joints.data[left_ear] = miro.constants.EAR_CALIB
        self.cos_joints.data[right_ear] = miro.constants.EAR_CALIB

        # robot name
        topic_base = "/" + os.getenv("MIRO_ROBOT_NAME") + "/"
        self.velocity = TwistStamped()

        # publish
        self.pub_cmd_vel = rospy.Publisher(topic_base + "control/cmd_vel", TwistStamped, queue_size=0)
        self.pub_kin = rospy.Publisher(topic_base + "control/kinematic_joints", JointState, queue_size=0)
        self.pub_cos = rospy.Publisher(topic_base + "control/cosmetic_joints", Float32MultiArray, queue_size=0)
        # subscribe
        self.sub_package = rospy.Subscriber(topic_base + "sensors/package", miro.msg.sensors_package,self.callback_package)
        print ("subscribe", topic_base + "sensors/package")
        # Subscribe to Camera topics
        self.cam_left_sub = rospy.Subscriber(topic_base + "sensors/caml/compressed", CompressedImage,self.cam_left_callback)
        self.cam_right_sub = rospy.Subscriber(topic_base + "sensors/camr/compressed", CompressedImage,self.cam_right_callback)
        # subscribe to microphone
        self.sub_mics = rospy.Subscriber(topic_base + "/sensors/mics", Int16MultiArray, self.callback_mics, queue_size=5, tcp_nodelay=True)
        print ("subscribe", topic_base + "/sensors/mics")

        self.cam_model = miro.utils.CameraModel()
        self.frame_w = 0
        self.frame_h = 0

    def kin_cos_init(self):
        time.sleep(0.01)
        self.pub_kin.publish(self.kin_joints)
        time.sleep(0.01)
        self.pub_cos.publish(self.cos_joints)

    def find_ball(self, colour_str, cam_id):
        if colour_str[0] != "#" and len(colour_str) != 7:
            print("colour choice should be a string in the form \"#RRGGBB\"")
            return
        if cam_id < 0 or cam_id > 1:
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

        return max_circle_norm

    def get_state(self):
        state = []
        leftCamera = self.find_ball("#0000FF", 0)
        rightCamera = self.find_ball("#0000FF", 1)

        print "left", leftCamera
        print "right", rightCamera

        if not leftCamera is None:
            if leftCamera[0] < 0:
                if leftCamera[2] < 50:
                    state.append(1)
                else: state.append(2)
            else:
                if leftCamera[2] < 50:
                    state.append(3)
                else:
                    state.append(4)
        else:
            state.append(0)

        if not rightCamera is None:
            if rightCamera[0] < 0:
                if rightCamera[2] < 50:
                    state.append(1)
                else:
                    state.append(2)
            else:
                if rightCamera[2] < 50:
                    state.append(3)
                else:
                    state.append(4)
        else:
            state.append(0)

        return state

    def step(self, action):
        # action = self.action_space[action_i]
        print "action", action

        # give the speed to go 0.2m and stop
        if action == 'PUSH':
            self.action_push()

        # turn 180 degree
        elif action == 'STEP_BACK':
            self.action_stepback()

        # turn 45 degree
        elif action == 'TURN_L_45':
            self.action_turn_l45()

        # turn 90 degree
        elif action == 'TURN_L_90':
            self.action_turn_l90()

        # turn -45 degree
        elif action == 'TURN_R_45':
            self.action_turn_r45()

        # turn -90 degree
        elif action == 'TURN_R_90':
            self.action_turn_r90()

        else:
            self.action_turn_180()

        new_state = self.get_state()
        # reach the goal have 1, else -1 reward
        if new_state[0] == self.goal[0] and new_state[1] == self.goal[1]:
            reward = 1
            done = True
            new_state = 'terminal'
        else:
            reward = -1
            done = False

        return new_state, reward, done

    def reset(self):
        return 0

    def action_push(self):
        start = datetime.datetime.now()

        while (self.sonar > 0.05):
            end = datetime.datetime.now()

            if (end - start).seconds > 1:
                self.velocity.twist.linear.x = 0.0
                self.velocity.twist.angular.z = 0.0
                self.pub_cmd_vel.publish(self.velocity)
                break
            # time.sleep(1)
            else:
                self.velocity.twist.linear.x = 0.1
                self.velocity.twist.angular.z = 0.0
                self.pub_cmd_vel.publish(self.velocity)

    def action_stepback(self):
        start = datetime.datetime.now()

        while (self.sonar < 0.05):
            end = datetime.datetime.now()
            if (end - start).seconds > 1:
                self.velocity.twist.linear.x = 0.0
                self.velocity.twist.angular.z = 0.0
                self.pub_cmd_vel.publish(self.velocity)
                break
            else:
                self.velocity.twist.linear.x = -0.05
                self.velocity.twist.angular.z = 0.0
                self.pub_cmd_vel.publish(self.velocity)

    def action_stop(self):
        start = datetime.datetime.now()

        while (True):
            end = datetime.datetime.now()
            if (end - start).seconds > 0:
                self.velocity.twist.linear.x = 0.0
                self.velocity.twist.angular.z = 0.0
                self.pub_cmd_vel.publish(self.velocity)
                break
            else:
                self.velocity.twist.linear.x = 0.0
                self.velocity.twist.angular.z = 0.0
                self.pub_cmd_vel.publish(self.velocity)

    def action_turn_l45(self):
        start = datetime.datetime.now()

        while (self.sonar > 0.05):
            end = datetime.datetime.now()
            if (end - start).seconds > 0:
                self.velocity.twist.linear.x = 0.0
                self.velocity.twist.angular.z = 0.0
                self.pub_cmd_vel.publish(self.velocity)
                break
            else:
                self.velocity.twist.linear.x = 0.0
                self.velocity.twist.angular.z = 0.79
                self.pub_cmd_vel.publish(self.velocity)

    def action_turn_l90(self):
        start = datetime.datetime.now()

        while (self.sonar > 0.05):
            end = datetime.datetime.now()
            if (end - start).seconds > 1:
                self.velocity.twist.linear.x = 0.0
                self.velocity.twist.angular.z = 0.0
                self.pub_cmd_vel.publish(self.velocity)
                break
            else:
                self.velocity.twist.linear.x = 0.0
                self.velocity.twist.angular.z = 0.79
                self.pub_cmd_vel.publish(self.velocity)

    def action_turn_r45(self):
        start = datetime.datetime.now()

        while (self.sonar > 0.05):
            end = datetime.datetime.now()
            if (end - start).seconds > 0:
                self.velocity.twist.linear.x = 0.0
                self.velocity.twist.angular.z = 0.0
                self.pub_cmd_vel.publish(self.velocity)
                break
            else:
                self.velocity.twist.linear.x = 0.0
                self.velocity.twist.angular.z = -0.79
                self.pub_cmd_vel.publish(self.velocity)

    def action_turn_r90(self):
        start = datetime.datetime.now()

        while (self.sonar > 0.05):
            end = datetime.datetime.now()
            if (end - start).seconds > 1:
                self.velocity.twist.linear.x = 0.0
                self.velocity.twist.angular.z = 0.0
                self.pub_cmd_vel.publish(self.velocity)
                break
            else:
                self.velocity.twist.linear.x = 0.0
                self.velocity.twist.angular.z = -0.79
                self.pub_cmd_vel.publish(self.velocity)

    def action_turn_180(self):
        start = datetime.datetime.now()

        while (True):
            end = datetime.datetime.now()
            if (end - start).seconds > 3:
                self.velocity.twist.linear.x = 0.0
                self.velocity.twist.angular.z = 0.0
                self.pub_cmd_vel.publish(self.velocity)
                break
            else:
                self.velocity.twist.linear.x = 0.0
                self.velocity.twist.angular.z = 0.79
                self.pub_cmd_vel.publish(self.velocity)

    def import_QTable(self):
        Q_table = pd.read_csv("/home/miro/mdk/bin/shared/qtable.csv")
        return Q_table

    def save_QTable(self,Q_table):
        Q_table.to_csv("/home/miro/mdk/bin/shared/qtable.csv")

    def import_ETable(self):
        E_table = pd.read_csv("/home/miro/mdk/bin/shared/etable.csv")
        return E_table

    def save_ETable(self,E_table):
        E_table.to_csv("/home/miro/mdk/bin/shared/etable.csv")

    def record_audio(self):
        # write output file
        file = wave.open(OUR_FILE_NAME, 'wb')
        file.setsampwidth(2)
        file.setframerate(MIC_SAMPLE_RATE)
        print "writing two channels to file (LEFT and RIGHT)..."
        file.setnchannels(2)
        x = np.reshape(self.outbuf[:, [0, 1]], (-1))
        for s in x:
            file.writeframes(struct.pack('<h', s))
        # close file
        file.close()
        print "wrote output file at", OUR_FILE_NAME

    def analyse_speech(self):
        # recognize the speech
        S2T = Speech2Text.SpeechToText(-1)  # use -1 for the microphone index on Miro as we have no direct stream access that Google would recognise for Miro's microphones

        print("Sentiment from audio files")
        print("**************************")
        print("Sentiment for client_audio.wav): ")
        return S2T.getSentiment(OUR_FILE_NAME)

if __name__ == "__main__":

    rospy.init_node("client_findball_shaping", anonymous=True)
    main = client_findball_shaping()
    main.loop()




