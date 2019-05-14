#!/usr/bin/python

################################################################

# python
import math
import numpy as np
import time
import miro2 as miro
from threading import Thread

# ROS
import rospy
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, Int16MultiArray
from geometry_msgs.msg import TwistStamped, Vector3
from sensor_msgs.msg import JointState, BatteryState, Imu, Range, CompressedImage, Image

#Open CV
import cv2
from cv_bridge import CvBridge, CvBridgeError

################################################################

class PlatformInterface:

	def __init__(self, name):

		# state
		self.m_ready = True

		# topic root
		self.topic_root = '/' + name + '/'

		#Configure ROS interface
		#Publishers
		self.velocity_pub = rospy.Publisher(self.topic_root + "control/cmd_vel", TwistStamped, queue_size=0)
		self.cosmetic_joints_pub = rospy.Publisher(self.topic_root + "control/cosmetic_joints", Float32MultiArray, queue_size=0)
		# self.illum_pub = rospy.Publisher(self.topic_root + "control/illum", UInt32MultiArray, queue_size=0)
		self.kinematic_joints_pub = rospy.Publisher(self.topic_root + "control/kinematic_joints", JointState, queue_size=0)
		# self.audio_tone_pub = rospy.Publisher(self.topic_root + "control/tone", UInt16MultiArray, queue_size=0)
		# self.param_pub = rospy.Publisher(self.topic_root + "control/params", Float32MultiArray, queue_size=0)
		self.push_pub = rospy.Publisher(self.topic_root + "core/push", miro.msg.push, queue_size=0)

		self.debug_image_pub = rospy.Publisher("/miro/blockly_circles", Image, queue_size=0)

		#Create objects to hold published data
		self.velocity = TwistStamped()

		self.kin_joints = JointState()
		self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
		self.kin_joints.position = [0.0, math.radians(34.0), 0.0, 0.0]

		self.cos_joints = Float32MultiArray()
		self.cos_joints.data = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]
		#
		# self.tone = UInt16MultiArray()
		# self.tone.data = [0, 0, 0]
		#
		# self.illum = UInt32MultiArray()
		# self.illum.data = [0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF]
		#
		# self.params = Float32MultiArray()
		# self.params.data = [721.0, 15.0]

		#Subscribe to sensors
		self.sensors_sub = rospy.Subscriber(self.topic_root + "sensors/package", miro.msg.sensors_package, self.sensors_callback)

		#Subscribe to Camera topics
		self.cam_left_sub = rospy.Subscriber(self.topic_root + "sensors/caml/compressed", CompressedImage, self.cam_left_callback)
		self.cam_right_sub = rospy.Subscriber(self.topic_root + "sensors/camr/compressed", CompressedImage, self.cam_right_callback)

		#Create Variables to store recieved data
		self.sonar_range = None
		self.light_array = [None, None, None, None]
		self.cliff_array = None
		self.head_touch = None
		self.body_touch = None

		#Arrays to hold image topics
		self.cam_left_image = None
		self.cam_right_image = None

		#Create object to convert ROS images to OpenCV format
		self.image_converter = CvBridge()

		#Create resource for controlling body_node
		self.pars = miro.utils.PlatformPars()
		self.cam_model = miro.utils.CameraModel()
		self.frame_width = 0
		self.frame_height = 0

	def sensors_callback(self, message):
		self.sonar_range = message.sonar.range
		self.light_array = message.light.data
		self.cliff_array = message.cliff.data
		self.head_touch =  message.touch_head.data
		self.body_touch =  message.touch_body.data

	def read_sonar_range(self):
		return self.sonar_range

	def read_light_level_list(self):
		return self.light_array

	def read_light_level(self, pos):
		if pos >= 0 and pos <4:
			return self.light_array[pos]

	def read_cliff_sensor(self, pos):
		if pos >= 0 and pos < 2:
			if self.cliff_array[pos] < 0.7:
				return True
			else:
				return False

	def read_cliff_sensor_list(self):
		return self.cliff_array

	def read_head_touch_sensors(self):
		return self.head_touch

	def read_body_touch_sensors(self):
		return self.body_touch

	def set_forward_speed(self, x):
		x = np.clip(x, -miro.constants.WHEEL_MAX_SPEED_M_PER_S, miro.constants.WHEEL_MAX_SPEED_M_PER_S)
		self.velocity.twist.linear.x = x


	def set_turn_speed(self, z_deg):
		z_rad = math.radians(z_deg)
		z_rad = np.clip(z_rad, -miro.constants.WHEEL_MAX_ANG_SPEED_RAD_PER_S, miro.constants.WHEEL_MAX_ANG_SPEED_RAD_PER_S)
		self.velocity.twist.angular.z = z_rad

	def set_neck(self, joint_index, degrees):
		if joint_index >= 1 and joint_index < 4:
			if joint_index == miro.constants.JOINT_LIFT:
				degrees = np.clip(degrees, 5, 60)
			elif joint_index == miro.constants.JOINT_PITCH:
				degrees = np.clip(degrees, -22, 8)
			elif joint_index == miro.constants.JOINT_YAW:
				degrees = np.clip(degrees, -60, 60)

			joint_radians = math.radians(degrees)
			self.kin_joints.position[joint_index] = joint_radians

	def set_ear(self, joint_index, pos):
		if 4 <= joint_index < 6:
			pos = np.clip(pos, 0.0, 1.0)
			self.cos_joints.data[joint_index] = pos

	def set_eyelid(self, joint_index, pos):
		if 2 <= joint_index < 4:
			pos = np.clip(pos, 0.0, 1.0)
			self.cos_joints.data[joint_index] = pos

	def set_tail(self, joint_index, pos):
		if 0 <= joint_index < 2:
			pos = np.clip(pos, 0.0, 1.0)
			self.cos_joints.data[joint_index] = pos

	def wag_tail_func(self, dur, rate):
		timeout_start = time.time()
		wag_phase = 0
		while time.time() < timeout_start + dur:
			wag_phase += np.pi / rate
			if wag_phase >= 2 * np.pi:
				wag_phase -= 2 * np.pi
			self.cos_joints.data[miro.constants.JOINT_WAG] = np.sin(wag_phase) * 0.5 + 0.5
			self.cosmetic_joints_pub.publish(self.cos_joints)
			time.sleep(0.1)

	def wag_tail(self, dur, rate):
		rate = np.clip(rate, 7, 30)
		wag_thread = Thread(target=self.wag_tail_func, args=(dur, rate))
		wag_thread.start()

	def cam_left_callback(self, ros_image):
		try:
			self.cam_left_image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "bgr8")
			im_h, im_w = self.cam_left_image.shape[:2]
			if self.frame_w != im_w and self.frame_h != im_h:
				self.frame_w, self.frame_h = im_w, im_h
				cam_model.set_frame_size(self.frame_w, self.frame_h)
		except CvBridgeError as e:
			print("Conversion of left image failed \n")
			print(e)

	def cam_right_callback(self, ros_image):
		try:
			self.cam_right_image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "bgr8")
			im_h, im_w = self.cam_right_image.shape[:2]
			if self.frame_w != im_w and self.frame_h != im_h:
				self.frame_w, self.frame_h = im_w, im_h
				cam_model.set_frame_size(self.frame_w, self.frame_h)
		except CvBridgeError as e:
			print("Conversion of right image failed \n")
			print(e)

	def update(self):

		# publish
		self.velocity_pub.publish(self.velocity)
		self.velocity = TwistStamped() # velocities are zeroed after use

		# publish
		self.kinematic_joints_pub.publish(self.kin_joints)

		# publish
		self.cosmetic_joints_pub.publish(self.cos_joints)

		# sleep (10Hz)
		time.sleep(0.1)

	def ready(self):

		# run update
		self.update()

		# return ready state
		return self.m_ready

	def exit(self):

		# set not ready
		self.m_ready = False
