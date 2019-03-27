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


def format_num(n, d=2):
	s = ("{:." + str(d) + "f}").format(n)
	if n >= 0:
		s = " " + s;
	return s

def wiggle(v, n, m):
	v = v + float(n) / float(m)
	if v > 2.0:
		v -= 2.0
	elif v > 1.0:
		v = 2.0 - v
	return v

def generate_argb(colour, bright):

	rgb_str = colour.to_string()
	useless, rgb_values = rgb_str.split("(")
	rgb_values, useless = rgb_values.split(")")
	r, g, b = rgb_values.split(",")
	r = int(r)
	g = int(g)
	b = int(b)
	return (int(bright) << 24) | (r << 16) | (g << 8) | b

def update_range(objs, lo, hi):
	for obj in objs:
		obj.set_lower(np.degrees(lo))
		obj.set_upper(np.degrees(hi))


#Generate a fake enum for joint arrays
tilt, lift, yaw, pitch = range(4)
droop, wag, left_eye, right_eye, left_ear, right_ear = range(6)
freq, volume, duration = range(3)
front_left, mid_left, rear_left, front_right, mid_right, rear_right = range(6)
closing = True
opening = False
right = True
left = False

class miro_gui:

	def __init__(self):

		# state
		self.step = 0

		#Load GUI from glade file
		self.builder = Gtk.Builder()
		self.builder.add_from_file("client_gui.glade")

		# update constants in builder
		update_range([
			self.builder.get_object("lift_adjustment"),
			self.builder.get_object("lift_meas_adjustment")
		], miro.constants.LIFT_RAD_MIN, miro.constants.LIFT_RAD_MAX)
		update_range([
			self.builder.get_object("yaw_adjustment"),
			self.builder.get_object("yaw_meas_adjustment")
		], miro.constants.YAW_RAD_MIN, miro.constants.YAW_RAD_MAX)
		update_range([
			self.builder.get_object("pitch_adjustment"),
			self.builder.get_object("pitch_meas_adjustment")
		], miro.constants.PITCH_RAD_MIN, miro.constants.PITCH_RAD_MAX)

		#Connect signals to handle callbacks from event
		self.builder.connect_signals(self)

		#Get Windows from GUI
		self.MainWindow = self.builder.get_object("MainWindow")
		self.CamWindow = self.builder.get_object("CamWindow")
		self.MicWindow = self.builder.get_object("MicWindow")
		self.MicScrolledWindow = self.builder.get_object("MicScrolledWindow")

		#Get Cam and Mic Buttons from GUI
		self.DisplayCamButton = self.builder.get_object("DisplayCamButton")
		self.DisplayMicButton = self.builder.get_object("DisplayMicButton")

		#Get Battery objects from GUI
		self.BatteryText = self.builder.get_object("BatteryText")
		self.BatteryBar = self.builder.get_object("BatteryBar")
		self.BatteryBar.add_offset_value(Gtk.LEVEL_BAR_OFFSET_LOW, 4.6)
		self.BatteryBar.add_offset_value(Gtk.LEVEL_BAR_OFFSET_HIGH, 4.8)
		#self.BatteryBar.add_offset_value(Gtk.LEVEL_BAR_OFFSET_FULL, 5.0)

		#Get Sonar objects from GUI
		self.SonarText = self.builder.get_object("SonarText")
		self.SonarBar = self.builder.get_object("SonarBar")

		#Get Light objects from GUI
		self.gui_LightBarFL = self.builder.get_object("LightBarFL")
		self.gui_LightBarFR = self.builder.get_object("LightBarFR")
		self.gui_LightBarRL = self.builder.get_object("LightBarRL")
		self.gui_LightBarRR = self.builder.get_object("LightBarRR")

		#Get Touch objects from GUI
		self.BodyTouchText = self.builder.get_object("BodyTouchText")
		self.HeadTouchText = self.builder.get_object("HeadTouchText")
		self.BodyTouchBars = []
		self.HeadTouchBars = []
		for i in range(14):
			temp = self.builder.get_object("BT" + str(i))
			self.BodyTouchBars.append(temp)
			temp = self.builder.get_object("HT" + str(i))
			self.HeadTouchBars.append(temp)

		#Get Accelerometer objects from GUI
		self.gui_AccBarBX = self.builder.get_object("AccBarBX")
		self.gui_AccBarBY = self.builder.get_object("AccBarBY")
		self.gui_AccBarBZ = self.builder.get_object("AccBarBZ")
		self.gui_AccBarBL2 = self.builder.get_object("AccBarBL2")
		self.gui_AccBarHX = self.builder.get_object("AccBarHX")
		self.gui_AccBarHY = self.builder.get_object("AccBarHY")
		self.gui_AccBarHZ = self.builder.get_object("AccBarHZ")
		self.gui_AccBarHL2 = self.builder.get_object("AccBarHL2")

		#Get Cliff objects from GUI
		self.gui_CliffBarL = self.builder.get_object("CliffBarL")
		self.gui_CliffBarR = self.builder.get_object("CliffBarR")

		#Get Wheel Speed objects from GUI
		self.gui_WheelSpeedOptoL = self.builder.get_object("WheelSpeedOptoL")
		self.gui_WheelSpeedOptoR = self.builder.get_object("WheelSpeedOptoR")
		self.gui_WheelSpeedEMFL = self.builder.get_object("WheelSpeedEMFL")
		self.gui_WheelSpeedEMFR = self.builder.get_object("WheelSpeedEMFR")

		self.VelControl = self.builder.get_object("VelControl")
		self.AngVelControl = self.builder.get_object("AngVelControl")
		self.UserControlVel = self.builder.get_object("UserControlVelocity")

		self.YawControl = self.builder.get_object("YawControl")
		self.LiftControl = self.builder.get_object("LiftControl")
		self.PitchControl = self.builder.get_object("PitchControl")
		self.ResetKinButton = self.builder.get_object("ResetKinButton")
		self.UserControlKin = self.builder.get_object("UserControlKin")

		self.LeftEyeControl = self.builder.get_object("LeftEyeControl")
		self.RightEyeControl = self.builder.get_object("RightEyeControl")
		self.BlinkButton = self.builder.get_object("BlinkButton")
		self.WiggleButton = self.builder.get_object("WiggleButton")
		self.LeftEarControl = self.builder.get_object("LeftEarControl")
		self.RightEarControl = self.builder.get_object("RightEarControl")
		self.WagControl = self.builder.get_object("WagControl")
		self.DroopControl = self.builder.get_object("DroopControl")
		self.WagRateControl = self.builder.get_object("WagRateControl")
		self.CosResetButton = self.builder.get_object("CosResetButton")
		self.UserControlCos = self.builder.get_object("UserControlCos")

		self.VelMeasured = self.builder.get_object("VelMeasured")
		self.AngVelMeasured = self.builder.get_object("AngVelMeasured")

		self.LiftMeasured = self.builder.get_object("LiftMeasured")
		self.YawMeasured = self.builder.get_object("YawMeasured")
		self.PitchMeasured = self.builder.get_object("PitchMeasured")
		self.gui_Camera = [
			self.builder.get_object("CameraLeft"),
			self.builder.get_object("CameraRight")
			]
		self.gui_ResolutionSelection = self.builder.get_object("ResolutionSelection")
		self.gui_CapResolutionSelection = self.builder.get_object("CapResolutionSelection")
		self.gui_CapFPSSelection = self.builder.get_object("CapFPSSelection")
		self.gui_MeasuredFPS = self.builder.get_object("MeasuredFPS")

		#Generate variables to store user input
		self.preset = False
		self.user_blink = 0
		self.user_waggle = 0
		self.user_wiggle = 0
		self.user_wag_rate = 0
		self.wag_phase = 0

	
		#HEAD
		self.head_plot = self.fig1.add_subplot(4,1,3)
		self.head_plot.set_ylim([-33000, 33000])
		self.head_plot.set_xlim([0, self.x_len])
		self.head_xs = np.arange(0, self.x_len)
		self.head_plot.set_xticklabels([])
		self.head_plot.set_yticks([])
		self.head_plot.grid(which="both", axis="x")
		self.head_plot.set_ylabel("Head", rotation=0, ha="right")
		self.head_ys = np.zeros(self.x_len)
		self.head_line, = self.head_plot.plot(self.head_xs, self.head_ys, linewidth=0.5, color="g")


		# variables to store input data
		self.input_package = None
		self.input_camera = [None, None]
		self.t_input_camera = [[], []]
		self.input_mics = np.zeros((self.x_len, self.no_of_mics))

		#Create object to convert ROS images to OpenCV format
		self.image_converter = CvBridge()

		#Start timer to call function which updates GUI
		GObject.timeout_add(20, self.update_gui)
		GObject.timeout_add(20, self.update_images)

		#Create objects to hold published data
		self.velocity = TwistStamped()

		self.kin_joints = JointState()
		self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
		self.kin_joints.position = [0.0, math.radians(34.0), 0.0, 0.0]

		self.cos_joints = Float32MultiArray()
		self.cos_joints.data = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]

		self.tone = UInt16MultiArray()
		self.tone.data = [0, 0, 0]

		self.illum = UInt32MultiArray()
		self.illum.data = [0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF]

		self.camera_zoom = None
		self.auto_camera_zoom = [0, 0] # determine zoom from first received frame
		self.frame_params = [180, '180w', 15]
		self.meas_fps = ["", ""]

		# set initial values
		self.on_ResetCosButton_clicked()
		self.on_LEDReset_clicked()

		# display GUI
		self.MainWindow.show_all()

		# robot name
		topic_base = "/" + os.getenv("MIRO_ROBOT_NAME") + "/"

		# publishers
		self.pub_cmd_vel = rospy.Publisher(topic_base + "control/cmd_vel", TwistStamped, queue_size=0)
		self.pub_cos = rospy.Publisher(topic_base + "control/cosmetic_joints", Float32MultiArray, queue_size=0)
		self.pub_illum = rospy.Publisher(topic_base + "control/illum", UInt32MultiArray, queue_size=0)
		self.pub_kin = rospy.Publisher(topic_base + "control/kinematic_joints", JointState, queue_size=0)
		self.pub_tone = rospy.Publisher(topic_base + "control/tone", UInt16MultiArray, queue_size=0)
		self.pub_command = rospy.Publisher(topic_base + "control/command", String, queue_size=0)

		# subscribers
		self.sub_package = rospy.Subscriber(topic_base + "sensors/package",
					miro.msg.sensors_package, self.callback_package)
		self.sub_mics = rospy.Subscriber(topic_base + "sensors/mics",
					Int16MultiArray, self.callback_mics)
		self.sub_caml = rospy.Subscriber(topic_base + "sensors/caml/compressed",
					CompressedImage, self.callback_caml)
		self.sub_camr = rospy.Subscriber(topic_base + "sensors/camr/compressed",
					CompressedImage, self.callback_camr)

################################################# VELOCITY CONTROL ###########################################################

	def on_VelControl_change_value(self, *args):
		self.velocity.twist.linear.x = self.VelControl.get_value()

	def on_AngVelControl_change_value(self, *args):
		self.velocity.twist.angular.z = self.AngVelControl.get_value()

	def on_ResetBodyVel_clicked(self, *args):
		self.VelControl.set_value(0.0)
		self.velocity.twist.linear.x = self.VelControl.get_value()
		self.AngVelControl.set_value(0.0)
		self.velocity.twist.angular.z = self.AngVelControl.get_value()

############################################### KINEMATICS CONTROL ###########################################################

	def on_ResetKinButton_clicked(self, *args):
		self.LiftControl.set_value(34.0)
		self.YawControl.set_value(0.0)
		self.PitchControl.set_value(0.0)
		self.kin_joints.position[lift] = math.radians(self.LiftControl.get_value())
		self.kin_joints.position[pitch] = math.radians(self.PitchControl.get_value())
		self.kin_joints.position[yaw] = math.radians(self.YawControl.get_value())

	def on_LiftControl_change_value(self, *args):
		self.kin_joints.position[lift] = math.radians(self.LiftControl.get_value())

	def on_PitchControl_change_value(self, *args):
		self.kin_joints.position[pitch] = math.radians(self.PitchControl.get_value())

	def on_YawControl_change_value(self, *args):
		self.kin_joints.position[yaw] = math.radians(self.YawControl.get_value())

############################################### COSMETICS CONTROL ###########################################################
##################################### Measured Velocities + Angles ##########################################################

	#Returning true from change_value prevents user from altering slider value
	def on_VelMeasured_change_value(self, *args):
		return True

	def on_AngVelMeasured_change_value(self, *args):
		return True

	def on_LiftMeasured_change_value(self, *args):
		return True

	def on_YawMeasured_change_value(self, *args):
		return True

	def on_PitchMeasured_change_value(self, *args):
		return True

	def on_SonarBar_change_value(self, *args):
		return True
 #############################################################################################################################

	def on_ButtonJpegLo_clicked(self, *args):
		cmd = "jpeg=10"
		self.pub_command.publish(cmd)

	def on_ButtonJpegMed_clicked(self, *args):
		cmd = "jpeg=60"
		self.pub_command.publish(cmd)

	def on_ButtonJpegHi_clicked(self, *args):
		cmd = "jpeg=80"
		self.pub_command.publish(cmd)

	def on_ResolutionSelection_changed(self, *args):
		self.camera_zoom = self.gui_ResolutionSelection.get_active_text()

	def parse_resolution(self, res_string):
		width_string, height_string = res_string.split("x", 1)
		width = int(width_string)
		height = int(height_string)
		return width, height

	def on_SendCamParamsButton_clicked(self, *args):
		cmd = "frame=" + self.frame_params[1] + "@" + str(self.frame_params[2])
		self.pub_command.publish(cmd)

		# set camera zoom to auto so it sets automatically when next frame arrives
		self.auto_camera_zoom = [self.frame_params[0], time.time()]

	def on_CapResolutionSelection_changed(self, *args):

		res_text = self.gui_CapResolutionSelection.get_active_text()
		cap_resolution_w, cap_resolution_h = self.parse_resolution(res_text)
		self.frame_params[0] = cap_resolution_h

		# convert to bridge format
		if (cap_resolution_w * 3) / 4 == cap_resolution_h:
			cap_resolution = str(cap_resolution_h) + "s"
		else:
			cap_resolution = str(cap_resolution_h) + "w"
		self.frame_params[1] = cap_resolution

	def on_CapFPSSelection_change_value(self, *args):
		self.frame_params[2] = int(self.gui_CapFPSSelection.get_value())

################################################################
## ROS CALLBACKS

	def callback_package(self, msg):

		# store for processing in update_gui
		self.input_package = msg

	def callback_mics(self, data):

		# discard data if GUI not displayed
		if not self.DisplayMicButton.get_active():
			return

		# save microphone data to rolling buffer for later processing
		# reshape into 4 x 500 array
		data = np.asarray(data.data)
		data = np.transpose(data.reshape((self.no_of_mics, 500)))
		data = np.flipud(data)
		# array is [ L, R, H, B] mics
		# add to top of buffer
		self.input_mics = np.vstack((data, self.input_mics[:self.x_len-500,:]))

	def do_auto_camera_zoom(self, image_height):

		if image_height <= 240:
			self.gui_ResolutionSelection.set_active(2)
		elif image_height > 600:
			self.gui_ResolutionSelection.set_active(0)
		else:
			self.gui_ResolutionSelection.set_active(1)
		self.on_ResolutionSelection_changed()

	def do_auto_camera_res(self, shape):

		l = ['240x180', '320x180', '320x240', '480x360', '640x360', '960x720', '1280x720']
		s = str(shape[1]) + "x" + str(shape[0])
		for i in range(len(l)):
			if s == l[i]:
				self.gui_CapResolutionSelection.set_active(i)
				return

	def callback_cam(self, ros_image, index):

		# discard data if GUI not displayed
		if not self.CamWindow.get_property("visible"):
			self.t_input_camera = [[], []]
			return

		# silently (ish) handle corrupted JPEG frames
		try:

			# convert compressed ROS image to raw CV image
			image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "rgb8")

			# set camera zoom automatically if has not been set already
			if not self.auto_camera_zoom is None:
				h = self.auto_camera_zoom[0]
				dt = time.time() - self.auto_camera_zoom[1]
				if h == 0:
					# initial state, set from first received frame regardless
					self.do_auto_camera_zoom(image.shape[0])
					self.auto_camera_zoom = None
					# for initial frame, also set resolution selector
					self.do_auto_camera_res(image.shape)
				elif dt > 4.0:
					self.auto_camera_zoom = None
				elif abs(image.shape[0] - h) < 32:
					self.do_auto_camera_zoom(h)
					self.auto_camera_zoom = None

			# do zoom
			if self.camera_zoom == "0.5x":
				image = cv2.resize(image, (int(image.shape[1] * 0.5), int(image.shape[0] * 0.5)))
			elif self.camera_zoom == "2x":
				image = cv2.resize(image, (int(image.shape[1] * 2.0), int(image.shape[0] * 2.0)))

			# store image for display
			self.input_camera[index] = image

		except CvBridgeError as e:

			# swallow error, silently
			#print(e)
			pass

	def callback_caml(self, ros_image):

		self.callback_cam(ros_image, 0)

	def callback_camr(self, ros_image):

		self.callback_cam(ros_image, 1)

## MAIN

if __name__ == "__main__":
	main = miro_gui()
	rospy.init_node("miro_gui")
	Gtk.main()




