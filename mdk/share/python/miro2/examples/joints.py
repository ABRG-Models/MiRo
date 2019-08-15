#!/usr/bin/python

# These message definitions tell ROSPy how to encode and decode data for MiRo's ROS nodes
# See http://labs.consequentialrobotics.com/miro-e/docs/index.php?page=Technical_Interfaces_ROS_interface for details
# on data types used by each node and some examples
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, Int16MultiArray, String
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, BatteryState, Imu, Range, CompressedImage

# Other imports
import os
import math
import rospy
import numpy as np
import miro2 as miro

# Set robot name
topic_root = "/" + os.getenv("MIRO_ROBOT_NAME")

# Python needs to initialise a ROS node for publishing data
rospy.init_node("python_pubisher", anonymous=True)

# Define ROS publisher
pub_kin = rospy.Publisher(topic_root + "/control/kinematic_joints", JointState, queue_size=0)

# Create object to hold published data
kin_joints = JointState()
kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
kin_joints.position = [0.0, math.radians(34.0), 0.0, 0.0]

# Generate fake enumerator for joint arrays
tilt, lift, yaw, pitch = range(4)

# I haven't checked if these random number generators outputs reasonable or valid angles
# Minimum and maximum values are listed in mdk/share/python/miro2/constants.py
lift_val = np.random.uniform(-20, 20, 1)
pitch_val = np.random.uniform(-20, 20, 1)
yaw_val = np.random.uniform(-20, 20, 1)

# Set joint values
kin_joints.position[lift] = math.radians(lift_val)
kin_joints.position[pitch] = math.radians(pitch_val)
kin_joints.position[yaw] = math.radians(yaw_val)

# Publish joint changes
while True:
	pub_kin.publish(kin_joints)

# Take a look at the client_gui.py code (mdk/bin/shared/client_gui.py) for a more detailed use of ROSPy