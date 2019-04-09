import rospy

# MiRo-E ROS interfaces
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, UInt32, Int16MultiArray, String
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, BatteryState, Imu, Range, CompressedImage
import miro2 as miro


# TODO:
# Add definitions for physical or sim. robot
class MiroClient:

	def callback_core_affect(self, object):

		print(object)

		if not self.active:
			return

		# store object
		self.core_affect = object

	def __init__(self):

		# set inactive
		self.active = False

		topic_root = "/miro"

		self.sub_core_affect = rospy.Subscriber(topic_root + "/core/affect", miro.msg.affect_state,
											self.callback_core_affect)

		# # Define defaults
		# self.core_affect = None