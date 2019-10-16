#	@section COPYRIGHT
#	Copyright (C) 2019 Consequential Robotics Ltd
#	
#	@section AUTHOR
#	Consequential Robotics http://consequentialrobotics.com
#	
#	@section LICENSE
#	For a full copy of the license agreement, and a complete
#	definition of "The Software", see LICENSE in the MDK root
#	directory.
#	
#	Subject to the terms of this Agreement, Consequential
#	Robotics grants to you a limited, non-exclusive, non-
#	transferable license, without right to sub-license, to use
#	"The Software" in accordance with this Agreement and any
#	other written agreement with Consequential Robotics.
#	Consequential Robotics does not transfer the title of "The
#	Software" to you; the license granted to you is not a sale.
#	This agreement is a binding legal agreement between
#	Consequential Robotics and the purchasers or users of "The
#	Software".
#	
#	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY
#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#	

################################################################

# python
import os
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

#interface
from audio import *
from vision import *

################################################################

#Should import common.py from share/web?
DIR_STATE=os.getenv('MIRO_DIR_STATE')
FILE_WEB_CMD=DIR_STATE + "/client_web.cmd"

class PlatformInterface:

	def __init__(self):

		#Initialise
		self.clap_detector = ClapDetector()
		self.vision = Vision()

		# state
		self.m_ready = True
		self.wagging = False
		self.web_cmd = ""

		# topic root
		self.topic_root = '/' + os.getenv("MIRO_ROBOT_NAME") + '/'

		#Configure ROS interface
		#Publishers
		self.velocity_pub = rospy.Publisher(self.topic_root + "control/cmd_vel", TwistStamped, queue_size=0)
		self.cosmetic_joints_pub = rospy.Publisher(self.topic_root + "control/cosmetic_joints", Float32MultiArray, queue_size=0)
		self.illum_pub = rospy.Publisher(self.topic_root + "control/illum", UInt32MultiArray, queue_size=0)
		self.kinematic_joints_pub = rospy.Publisher(self.topic_root + "control/kinematic_joints", JointState, queue_size=0)
		self.audio_tone_pub = rospy.Publisher(self.topic_root + "control/tone", UInt16MultiArray, queue_size=0)
		# self.param_pub = rospy.Publisher(self.topic_root + "control/params", Float32MultiArray, queue_size=0)
		self.push_pub = rospy.Publisher(self.topic_root + "core/push", miro.msg.push, queue_size=0)

		#Create objects to hold published data
		self.velocity = TwistStamped()

		self.kin_joints = JointState()
		self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
		self.kin_joints.position = [0.0, miro.constants.LIFT_RAD_CALIB, 0.0, 0.0]

		self.cos_joints = Float32MultiArray()
		self.cos_joints.data = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]

		self.tone = UInt16MultiArray()
		self.tone.data = [0, 0, 0]

		self.illum = UInt32MultiArray()
		self.illum.data = [0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF]

		# self.params = Float32MultiArray()
		# self.params.data = [721.0, 15.0]

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
		self.frame_w = 0
		self.frame_h = 0

		#Timer variables
		self.pause_flag = False
		self.timer_end_time = 0.0
		self.time_now = 0.0

		#Start thread
		self.updated = False
 		self.update_thread = Thread(target=self.update)
 		self.update_thread.start()
		self.thread_running = True

		#Subscribe to sensors
		self.touch_body_sub = rospy.Subscriber(self.topic_root + "sensors/touch_body", UInt16, self.touch_body_callback, tcp_nodelay=True)
		self.touch_head_sub = rospy.Subscriber(self.topic_root + "sensors/touch_head", UInt16, self.touch_head_callback, tcp_nodelay=True)
		self.mics_sub = rospy.Subscriber(self.topic_root + "sensors/mics", Int16MultiArray, self.mics_callback, tcp_nodelay=True)

		#Subscribe to Camera topics
		self.cam_left_sub = rospy.Subscriber(self.topic_root + "sensors/caml/compressed", CompressedImage, self.cam_left_callback, tcp_nodelay=True)
		self.cam_right_sub = rospy.Subscriber(self.topic_root + "sensors/camr/compressed", CompressedImage, self.cam_right_callback, tcp_nodelay=True)

		# last subscription is to sensors_package because that drives the clock
		self.sensors_sub = rospy.Subscriber(self.topic_root + "sensors/package", miro.msg.sensors_package, self.sensors_callback, tcp_nodelay=True)

	def yield_cpu(self):

		# just yield the CPU briefly
		time.sleep(0.01)

	def get_time(self):

		# typically, we want to use the physical clock
		return self.time_now

		# we can use the wallclock, as an alternative, but it's not right
		#return time.time()

	def sensors_callback(self, message):

		# we use this call to advance the clock
		self.time_now += 0.02

		self.sonar_range = message.sonar.range
		self.light_array = message.light.data
		self.cliff_array = message.cliff.data

		#These sensors need their own streams so they can be spoofed
		#self.head_touch =  message.touch_head.data
		#self.body_touch =  message.touch_body.data

	def touch_body_callback(self, message):
		self.body_touch = message.data

	def touch_head_callback(self, message):
		self.head_touch = message.data

	def mics_callback(self, message):
		self.clap_detector.detect_clap(message.data, self.time_now)

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
		#return self.head_touch

		# new list format
		return [bool(int(x)) for x in '{:014b}'.format(self.head_touch)]

	def read_body_touch_sensors(self):
		#return self.body_touch

		# new list format
		return [bool(int(x)) for x in '{:014b}'.format(self.body_touch)]

	def find_ball(self, colour_str, cam_id, prop_index=None):
		self.pause()
		if colour_str[0] != "#" and len(colour_str) != 7:
			print("colour choice should be a string in the form \"#RRGGBB\"")
			return None
		if cam_id < 0 or cam_id > 1:
			print("camera index should be 0 or 1")
			return None

		ball = self.vision.detect_ball(colour_str, cam_id)
		if ball is None:
			return None

		if not prop_index is None:
			if prop_index < 3:
				return ball[prop_index]

		# if not a recognised prop_index, return [x, y, size]
		return ball

	def clear_ball(self, cam_id):
		if cam_id < 0 or cam_id > 1:
			return
		self.vision.clear(cam_id)

	def clap(self):
		if self.clap_detector.have_seen_clap():
			return True
		else:
			return False

	def time_since_clap(self):
		if self.clap_detector.time_at_clap != None:
			return self.get_time() - self.clap_detector.time_at_clap
		else:
			return 60.0 # just return some large value

	def clap_in_previous(self, secs):
		if self.time_since_clap() < secs:
			return True
		else:
			return False

	def set_forward_speed(self, x):
		self.pause()
		x = np.clip(x, -miro.constants.WHEEL_MAX_SPEED_M_PER_S, miro.constants.WHEEL_MAX_SPEED_M_PER_S)
		self.velocity.twist.linear.x = x


	def set_turn_speed(self, z_deg):
		self.pause()
		z_rad = math.radians(z_deg)
		z_rad = np.clip(z_rad, -miro.constants.WHEEL_MAX_ANG_SPEED_RAD_PER_S, miro.constants.WHEEL_MAX_ANG_SPEED_RAD_PER_S)
		self.velocity.twist.angular.z = z_rad

	def set_neck(self, joint_index, degrees):
		self.pause()
		if joint_index >= 1 and joint_index < 4:
			if joint_index == miro.constants.JOINT_LIFT:
				degrees = np.clip(degrees, 5, 60)
			elif joint_index == miro.constants.JOINT_PITCH:
				degrees = np.clip(degrees, -22, 8)
			elif joint_index == miro.constants.JOINT_YAW:
				degrees = np.clip(degrees, -60, 60)

			joint_radians = math.radians(degrees)
			self.kin_joints.position[joint_index] = joint_radians

	def set_joint(self, joint_index, pos):
		self.pause()
		if 0 <= joint_index < 6:
			pos = np.clip(pos, 0.0, 1.0)
			self.cos_joints.data[joint_index] = pos

	def wag_tail_func(self, dur, rate):

		t_end_wag = self.get_time() + dur
		wag_phase = 0
		self.wagging = True

		while self.get_time() < t_end_wag:
			self.pause()
			wag_phase += np.pi / rate
			if wag_phase >= 2 * np.pi:
				wag_phase -= 2 * np.pi
			self.cos_joints.data[miro.constants.JOINT_WAG] = np.sin(wag_phase) * 0.5 + 0.5
			self.yield_cpu()

		self.wagging = False

	def wag_tail(self, dur, rate):
		self.pause()
		rate = np.clip(rate, 7, 30)
		wag_thread = Thread(target=self.wag_tail_func, args=(dur, rate))
		wag_thread.start()

	def play_tone(self, audio_freq, audio_vol, audio_dur_secs):
		self.pause()
		audio_freq = np.clip(audio_freq, 200, 2000)
		audio_vol = np.clip(audio_vol, 0, 255)
		audio_dur = int(audio_dur_secs*50)
		self.tone.data[miro.constants.TONE_FREQ] = audio_freq
		self.tone.data[miro.constants.TONE_DUR] = audio_dur
		self.tone.data[miro.constants.TONE_VOL] = audio_vol
		self.audio_tone_pub.publish(self.tone)

	def control_led(self, led_pos, colour_str, brightness):
		self.pause()
		led_pos = np.clip(led_pos, 0, 5)
		brightness = np.clip(brightness, 0, 255)
		r = int(colour_str[1:3], 16)
		g = int(colour_str[3:5], 16)
		b = int(colour_str[5:7], 16)
		led_value = (int(brightness) << 24) | (r << 16) | (g << 8) | b
		self.illum.data[led_pos] = led_value

	def sleep(self, user_secs):

		# sleep end time
		t_end_sleep = self.get_time() + user_secs

		while self.get_time() < t_end_sleep:
			self.pause()
			self.yield_cpu()

	def start_timer(self, user_secs):
		self.pause()
		self.timer_end_time = self.get_time() + user_secs

	def timer(self):
		if self.get_time() < self.timer_end_time:
			self.yield_cpu()
			return True
		else:
			self.timer_end_time = 0.0
			return False

	def cam_left_callback(self, ros_image):
		self.vision.process(ros_image, 0)

	def cam_right_callback(self, ros_image):
		self.vision.process(ros_image, 1)

	def update(self):

		t_loop = 0.0

		while(self.m_ready or self.wagging):

			# wait for boundary
			if self.time_now < t_loop:

				# yield
				self.yield_cpu()

				# restart loop
				continue

			# check pause
			self.pause()

			# publish
			self.velocity_pub.publish(self.velocity)
			#self.velocity = TwistStamped() # velocities are zeroed after use

			# publish
			self.kinematic_joints_pub.publish(self.kin_joints)

			# publish
			self.cosmetic_joints_pub.publish(self.cos_joints)

			# publish
			self.illum_pub.publish(self.illum)

			# Change flag to allow user control loop to run
			self.updated = True

			# advance at 10Hz
			t_loop += 0.1

		# mark exit
		self.thread_running = False

	def ready(self):

		# wait for boundary
		while not self.updated:
			self.yield_cpu()

		# clear ready state
		self.updated = False

		# return ready state
		return self.m_ready

	def pause(self):

		# currently, we do not support pause
		pass

		"""
		#Time at start of pause
		pause_start = self.get_time()

		#Check for Pause Flag
		while (self.m_ready or self.wagging):
			# Check for presence of pause command
			if os.path.exists(FILE_WEB_CMD):
				with open(FILE_WEB_CMD, "r", os.O_NONBLOCK) as file:
					self.web_cmd = file.read()
					file.close()
			else:
				self.web_cmd = ""

			if self.web_cmd == "pause":
				self.pause_flag = True
			else:
				break

			time.sleep(0.1)

		#Calculate pause time
		if self.pause_flag:
			pause_time = self.get_time() - pause_start
			self.timer_end_time = self.timer_end_time + pause_time
			self.pause_flag = False
		"""

	def exit(self):

		# silently allow double call - this is only necessary because
		# currently the code generation for robot blockly is a bit
		# weird, and we can't help but call this twice in case of a
		# successful script completion. TODO: fix the code generation,
		# then we can remove this behaviour (we should perhaps generate
		# a warning, then, instead).
		if self.m_ready:

			# let thread exit
			self.m_ready = False

			# wait for it to do so
			print "[robot interface] waiting for thread exit..."
			while self.thread_running:
				time.sleep(0.1)
			print "[robot interface] OK"

			# disable all subs
			self.sensors_sub.unregister()
			self.touch_body_sub.unregister()
			self.touch_head_sub.unregister()
			self.mics_sub.unregister()
			self.cam_left_sub.unregister()
			self.cam_right_sub.unregister()

			# wait one second to hopefully catch any remaining
			# callbacks and avoid spurious error msgs at close
			time.sleep(1.0)
