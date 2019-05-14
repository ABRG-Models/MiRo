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

################################################################

#Should import common.py from share/web?
DIR_STATE=os.getenv('MIRO_DIR_STATE')
FILE_WEB_CMD=DIR_STATE + "/client_web.cmd"

class PlatformInterface:

	def __init__(self, name):

		# state
		self.m_ready = True
		self.wagging = False
		self.web_cmd = ""

		# topic root
		self.topic_root = '/' + name + '/'

		#Configure ROS interface
		#Publishers
		self.velocity_pub = rospy.Publisher(self.topic_root + "control/cmd_vel", TwistStamped, queue_size=0)
		self.cosmetic_joints_pub = rospy.Publisher(self.topic_root + "control/cosmetic_joints", Float32MultiArray, queue_size=0)
		self.illum_pub = rospy.Publisher(self.topic_root + "control/illum", UInt32MultiArray, queue_size=0)
		self.kinematic_joints_pub = rospy.Publisher(self.topic_root + "control/kinematic_joints", JointState, queue_size=0)
		self.audio_tone_pub = rospy.Publisher(self.topic_root + "control/tone", UInt16MultiArray, queue_size=0)
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

		self.tone = UInt16MultiArray()
		self.tone.data = [0, 0, 0]

		self.illum = UInt32MultiArray()
		self.illum.data = [0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF]

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
		self.frame_w = 0
		self.frame_h = 0

		#Timer variables
		self.pause_flag = False
		self.timer_end_time = 0.0

		#Start thread
		self.updated = False
 		self.update_thread = Thread(target=self.update)
 		self.update_thread.start()


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

	def find_ball(self, colour_str, cam_id, prop):
		self.pause()
		if colour_str[0] != "#" and len(colour_str) != 7:
			print("colour choice should be a string in the form \"#RRGGBB\"")
			return
		if cam_id < 0 or cam_id > 1:
			return
		if prop < 0 or prop > 2:
			return

		#create colour code from user selected colour
		red = int(colour_str[1:3], 16)
		green = int(colour_str[3:5], 16)
		blue = int(colour_str[5:7], 16)
		bgr_colour = np.uint8([[[blue, green, red]]])
		hsv_colour = cv2.cvtColor(bgr_colour, cv2.COLOR_BGR2HSV)

		#extract boundaries for masking image
		target_hue = hsv_colour[0,0][0]
		lower_bound = np.array([target_hue-20, 70, 70])
		upper_bound = np.array([target_hue+20, 255, 255])

		if np.shape(self.cam_left_image) != () and np.shape(self.cam_right_image) != ():
			#convert camera image to HSV colour space
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

		#mask image
		mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
		seg = mask

		# Do some processing
		seg = cv2.GaussianBlur(seg, (11,11), 0)
		seg = cv2.erode(seg, None, iterations=2)
		seg = cv2.dilate(seg, None, iterations=2)

		# get circles
		circles = cv2.HoughCircles(seg, cv2.HOUGH_GRADIENT, 1, 40, param1=10, param2=20,minRadius=0, maxRadius=0)

		# Get largest circle
		max_circle = None
		max_circle_norm = [None, None, None]
		if circles is not None:
			self.max_rad = 0
			circles = np.uint16(np.around(circles))

			for c in circles[0,:]:
				cv2.circle(seg, (c[0], c[1]), c[2], (0, 255, 0), 2)

				if c[2] > self.max_rad:
					self.max_rad = c[2]
					max_circle = c
					max_circle_norm[0] = int(round(((max_circle[0] - im_centre_w) / im_centre_w) * 100.0))
					max_circle_norm[1] = int(round(-((max_circle[1] - im_centre_h) / im_centre_h) * 100.0))
					max_circle_norm[2] = int(round((max_circle[2]/im_centre_w)*100.0))

				#Debug Only
				cv2.circle(output, (max_circle[0], max_circle[1]), max_circle[2], (0, 255, 0), 2)
				cv2.circle(output, (max_circle[0], max_circle[1]), 1, (0, 255, 0), 2)
				location_str = "x: " + str(max_circle_norm[0]) + "," + "y: " + str(max_circle_norm[1]) + "," + "r: " + str(max_circle[2])
				text_y_offset = 18
				for i, line in enumerate(location_str.split(",")):
					text_y = max_circle[1] - text_y_offset + i*text_y_offset
					cv2.putText(output, line, (max_circle[0]+5, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
					cv2.putText(output, line, (max_circle[0]+5, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

		else:
			return None

		debug_image = self.image_converter.cv2_to_imgmsg(output, "bgr8")
		self.debug_image_pub.publish(debug_image)
		return max_circle_norm[prop]

	def get_ball_loc(self, colour_str):
		self.pause()
		if colour_str[0] != "#" and len(colour_str) != 7:
			print("colour choice should be a string in the form \"#RRGGBB\"")
			return

		if np.shape(self.cam_left_image) == () or np.shape(self.cam_right_image) == ():
			return

		#create colour code from user selected colour
		red = int(colour_str[1:3], 16)
		green = int(colour_str[3:5], 16)
		blue = int(colour_str[5:7], 16)
		bgr_colour = np.uint8([[[blue, green, red]]])
		hsv_colour = cv2.cvtColor(bgr_colour, cv2.COLOR_BGR2HSV)

		#extract boundaries for masking image
		target_hue = hsv_colour[0,0][0]
		lower_bound = np.array([target_hue-20, 70, 70])
		upper_bound = np.array([target_hue+20, 255, 255])

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

			#mask image
			mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
			seg = mask

			# Do some processing
			seg = cv2.GaussianBlur(seg, (11,11), 0)
			seg = cv2.erode(seg, None, iterations=2)
			seg = cv2.dilate(seg, None, iterations=2)

			# get circles
			circles = cv2.HoughCircles(seg, cv2.HOUGH_GRADIENT, 1, 40, param1=10, param2=33, minRadius=0, maxRadius=0)

			#Search through circles for largest
			if circles is not None:
				circles = np.uint16(np.around(circles))
				for circle in circles[0,:]:
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

	def pixel_to_point(self, pixel_loc, range, cam_index):
		self.pause()
		if pixel_loc != None:
			view_line = self.cam_model.p2v(pixel_loc) #Transform to view line in CAM
			point_head = self.cam_model.v2oh(cam_index, view_line, range)
			return point_head
		else:
			return None

	def push_to_point(self, point):
		self.pause()
		push_msg = miro.msg.push()
		push_loc = miro.utils.get("LOC_SONAR_FOVEA_HEAD")
		push_msg.pushpos = Vector3(push_loc[0], push_loc[1], push_loc[2])
		push_vec = point - push_loc
		push_vec /= np.linalg.norm(push_vec)
		push_vec *= 0.1
		push_msg.pushvec = Vector3(push_vec[0], push_vec[1], push_vec[2])
		push_msg.flags = miro.constants.PUSH_FLAG_IMPULSE
		push_msg.link = miro.constants.LINK_HEAD
		self.push_pub.publish(push_msg)

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
		timeout_start = time.time()
		wag_phase = 0
		self.wagging = True
		while time.time() < timeout_start + dur:
			self.pause()
			wag_phase += np.pi / rate
			if wag_phase >= 2 * np.pi:
				wag_phase -= 2 * np.pi
			self.cos_joints.data[miro.constants.JOINT_WAG] = np.sin(wag_phase) * 0.5 + 0.5
			time.sleep(0.1)
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
		self.start_timer(user_secs)
		while time.time() < self.timer_end_time:
			time.sleep(0.01)

	def start_timer(self, user_secs):
		self.pause()
		self.timer_end_time = time.time() + user_secs

	def timer(self):
		if time.time() < self.timer_end_time:
			time.sleep(0.01)
			return True
		else:
			self.timer_end_time = 0.0
			return False

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

	def update(self):
		while(self.m_ready or self.wagging):

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

			# sleep (10Hz)
			time.sleep(0.1)

	def ready(self):
		# Wait for thread to publish commands before executing user loop
		self.updated = False
		while self.updated != True:
			time.sleep(0.01)

		# return ready state
		return self.m_ready

	def pause(self):
		#Time at start of pause
		pause_start = time.time()

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
			pause_time = time.time() - pause_start
			self.timer_end_time = self.timer_end_time + pause_time
			self.pause_flag = False

	def exit(self):

		self.updated = False
		while self.updated != True:
			time.sleep(0.01)

		# set not ready
		self.m_ready = False
		self.web_cmd = ""

		#stop update thread
		self.update_thread.join()
