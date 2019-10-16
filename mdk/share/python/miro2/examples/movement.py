#!/usr/bin/python

# These message definitions tell ROSPy how to encode and decode data for MiRo's ROS nodes
# See http://labs.consequentialrobotics.com/miro-e/docs/index.php?page=Technical_Interfaces_ROS_interface for details
# on data types used by each node and some examples
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, Int16MultiArray, String
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, BatteryState, Imu, Range, CompressedImage

# Other imports
import os
import rospy
import numpy as np
import miro2 as miro

# Set robot name
topic_root = "/" + os.getenv("MIRO_ROBOT_NAME")

# Python needs to initialise a ROS node for publishing data
rospy.init_node("python_publisher", anonymous=True)

# Define ROS publisher
pub_cmd_vel = rospy.Publisher(topic_root + "/control/cmd_vel", TwistStamped, queue_size=10)

# Create object to hold published data
velocity = TwistStamped()

# Set left/right wheels speeds in m/s
l_val = np.random.uniform(-1, 1, 1)
r_val = np.random.uniform(-1, 1, 1)
wheel_speed = [l_val, r_val]

# Convert to command velocity
(dr, dtheta) = miro.utils.wheel_speed2cmd_vel(wheel_speed)

# Set velocity values
velocity.twist.linear.x = dr
velocity.twist.angular.z = dtheta

print wheel_speed

# Publish command velocity
while True:
	print(velocity)
	pub_cmd_vel.publish(velocity)

# Take a look at the client_gui.py code (mdk/bin/shared/client_gui.py) for a more detailed use of ROSPy
