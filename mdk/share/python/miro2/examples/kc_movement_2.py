#!/usr/bin/python

from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, Int16MultiArray, String
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, BatteryState, Imu, Range, CompressedImage

# Other imports
import os
import rospy
import numpy as np
import miro2 as miro
import time

from core import pars

# Set robot name
topic_root = "/" + os.getenv("MIRO_ROBOT_NAME")

# Python needs to initialise a ROS node for publishing data
rospy.init_node("python_publisher", anonymous=True)

# Define ROS publishers
pub_kin = rospy.Publisher(topic_root + '/control/kinematic_joints', JointState, queue_size=10)
pub_cmd_vel = rospy.Publisher(topic_root + '/control/cmd_vel', TwistStamped, queue_size=10)

CURRENT_ELEV = 0.785
TARGET_ELEV = 0.5
TARGET_AZIM = -1.5

# # # # #

# DemoSystem init
# Import core parameters (from client_demo line 221)
pars = pars.CorePars()

# ActionTemplate init
# Initialise private clock object (from action_types line 63)
clock = 0

# KC_M (from client_demo line 231)
kc_m = miro.utils.kc_interf.kc_miro()

# Create movement and orientation objects
velocity = TwistStamped()   # (from interface line 84)
kin_joints = JointState()   # (from interface line 86)

# # # # #

# From action_orient : start
# Called from action_types : descending (209)
# Called from node_action : tick (343)
# Called from client_demo : callback_sensors_package (406)

# get current gaze target in HEAD
# gaze_HEAD = miro.utils.kc_interf.kc_viewline_to_position(
gaze_HEAD = miro.utils.kc_interf.kc_view_to_HEAD(
	0.0,
	CURRENT_ELEV,    # Should be current gaze elevation
	pars.action.orient_gaze_target_radius
)

# get current gaze target in WORLD
gaze_i_WORLD = kc_m.changeFrameAbs(
	miro.constants.LINK_HEAD,
	miro.constants.LINK_WORLD,
	# miro.utils.kc_interf.kc_viewline_to_position(
	miro.utils.kc_interf.kc_view_to_HEAD(
		0.0,
		CURRENT_ELEV,    # Should be current gaze elevation
		pars.action.orient_gaze_target_radius
	)
)

# get desired gaze target in WORLD
gaze_f_WORLD = kc_m.changeFrameAbs(
	miro.constants.LINK_HEAD,
	miro.constants.LINK_WORLD,
	# miro.utils.kc_interf.kc_viewline_to_position(
	miro.utils.kc_interf.kc_view_to_HEAD(
		TARGET_AZIM,    # Azimuth
		TARGET_ELEV,    # Elevation
		pars.action.orient_gaze_target_radius
	)
)

# get change in gaze target across movement
dgaze_WORLD = gaze_f_WORLD - gaze_i_WORLD
# Debug
# print('Gaze difference: ' + str(dgaze_WORLD))

# decide pattern rate / time
elev = TARGET_ELEV - CURRENT_ELEV
rad = np.sqrt(np.square(TARGET_AZIM) + np.square(elev))
sec_ideal = rad * pars.action.orient_speed_sec_per_rad
steps_ideal = int(sec_ideal * pars.timing.tick_hz)
steps = np.clip(steps_ideal, pars.action.orient_min_steps, pars.action.orient_max_steps)

# start action clock
clock_f = steps


while clock < clock_f:

	# From action_orient : service
	# Called from action_types : descending (196)
	# Called from node_action : tick (343)
	# Called from client_demo : callback_sensors_package (406)

	# read clock
	x = float(clock) / float(clock_f)

	clock += 1
	# Debug
	# if x != 0.0:
	# 	print(x)

	# compute interim gaze target along a straight line (not quite an arc, but no matter...)
	gaze_x_WORLD = gaze_i_WORLD + x * dgaze_WORLD

	# Debug
	# if x != 0.0:
	# 	print('gaze_x_WORLD: ' + str(gaze_x_WORLD))

	# transform into HEAD for actioning as a push
	gaze_x_HEAD = kc_m.changeFrameAbs(
		miro.constants.LINK_WORLD,
		miro.constants.LINK_HEAD,
		gaze_x_WORLD
	)

	# correct range to gaze target range so we turn rather than shimmy
	# (we won't shimmy anyway with NO_TRANSLATION, but still...)
	gaze_x_HEAD *= pars.action.orient_gaze_target_radius / np.linalg.norm(gaze_x_HEAD)

	# apply push
	push = miro.utils.kc.KinematicPush()
	push.link = miro.constants.LINK_HEAD
	push.flags = 0 | miro.constants.PUSH_FLAG_IMPULSE | miro.constants.PUSH_FLAG_NO_TRANSLATION
	# | miro.constants.PUSH_FLAG_NO_TRANSLATION
	push.pos = gaze_HEAD
	push.vec = (gaze_x_HEAD - gaze_HEAD)
	# self.apply_push(push)

	# END action_orient : service
	# CONT node_action : tick (343)

	# add flags
	"""
	if pars.flags.BODY_ENABLE_TRANSLATION == 0:
		push.flags |= miro.constants.PUSH_FLAG_NO_TRANSLATION
	if pars.flags.BODY_ENABLE_ROTATION == 0:
		push.flags |= miro.constants.PUSH_FLAG_NO_ROTATION
	if pars.flags.BODY_ENABLE_NECK_MOVEMENT == 0:
		push.flags |= miro.constants.PUSH_FLAG_NO_NECK_MOVEMENT
		"""

	# <snip>

	# apply push to local kc
	kc_m.push(push)

	# END node_action : tick
	# CONT client_demo : callback_sensors_package (406)

	# <snip>

	# get config & dpose from kc
	config = kc_m.getConfig()
	dpose = kc_m.getPoseChange() * miro.constants.PLATFORM_TICK_HZ

	print(x, push.vec, dpose)

	kin_joints.position = config
	pub_kin.publish(kin_joints)

	velocity.twist.linear.x = dpose[0]
	velocity.twist.angular.z = dpose[1]
	pub_cmd_vel.publish(velocity)

	time.sleep(0.05)


