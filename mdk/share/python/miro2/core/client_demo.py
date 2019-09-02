#!/usr/bin/python
#
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

import os
import sys

import rospy
import sensor_msgs
import std_msgs
import geometry_msgs

# support
import pars
import miro2 as miro
from cv_bridge import CvBridge

# nodes
from node_lower import *
from node_decode import *
from node_affect import *
from node_express import *
from node_action import *
from node_loop import *
from node_detect_motion import *
from node_detect_ball import *
from node_detect_face import *
from node_detect_audio import *
from node_spatial import *
from node_face_recognition import *



class DemoPub:

	def __init__(self, pub, data_type):

		# if data_type is not None, instantiate a message
		if not data_type is None:
			msg = data_type()
		else:
			msg = None

		self.pub = pub
		self.msg = msg

	def publish(self):

		# if a msg was passed
		self.pub.publish(self.msg)



class DemoInput:

	def __init__(self):

		# instantiate
		self.sensors_package = None
		self.voice_state = None
		self.mics = None



class DemoState:

	def __init__(self, pars):

		# shared resources
		self.camera_model = miro.utils.camera_model.CameraModel(pars)

		# system
		self.tick = 0
		self.keep_running = True

		# 50Hz
		self.motors_active = False
		self.user_touch = 0.0
		self.light_mean = 0.0
		self.pet = 0.0
		self.stroke = 0.0
		self.jerk_head = 0.0
		self.jerk_body = 0.0
		self.emotion = None
		self.wakefulness = 0.0
		self.fovea_speed = 0.0
		self.halting = False
		self.action_target_value = None

		# loop feedback
		self.in_blink = 0.0
		self.in_cos_body = 0.0
		self.in_cos_head = 0.0
		self.in_motion = 0.0
		self.in_vocalising = 0.0
		self.in_making_noise = 0.0

		# cameras
		self.frame_raw = [None, None]
		self.frame_gry = [None, None]
		self.frame_mov = [None, None]
		self.frame_bal = [None, None]
		self.frame_pri = [None, None, None]

		# stimulus source information
		self.priority_peak = None

		# mics
		self.audio_events_for_spatial = []
		self.audio_events_for_50Hz = []

		# detected objects
		self.detect_ball = [None, None]
		# detect_face [faces_coordinate, image]
		self.detect_face = [[], []]
		self.primary_user = False

		# internal
		self.reconfigure_cameras = False
		self.reconfigured_cameras = False



class DemoOutput:

	def __init__(self):

		# instantiate
		self.cosmetic_joints = np.array([0, 0.5, 0.5, 0.5, 0.2, 0])
		self.illum = [0] * 6
		self.affect = None
		self.pushes = []
		self.tone = 0



class DemoNodes:

	def __init__(self, client_type):

		self.client_type = client_type

	def instantiate(self, sys):

		# instantiate
		if self.client_type == "main":
			self.lower = NodeLower(sys)
			self.affect = NodeAffect(sys)
			self.express = NodeExpress(sys)
			self.action = NodeAction(sys)
			self.loop = NodeLoop(sys)
			self.spatial = NodeSpatial(sys)

		# instantiate
		if self.client_type == "camera":
			self.decode = NodeDecode(sys)
			self.detect_motion = NodeDetectMotion(sys)
			self.detect_face = NodeDetectFace(sys)
			self.face_recognition = NodeFaceRecognition(sys)
			self.detect_ball = NodeDetectBall(sys)

		# instantiate
		if self.client_type == "mics":
			self.detect_audio = NodeDetectAudio(sys)


		# # instantiate
		# if self.client_type == "recognition":
		# 	self.decode = NodeDecode(sys)
		# 	self.detect_face = NodeDetectFace(sys)



	def tick(self):

		# tick
		if self.client_type == "main":
			self.lower.tick()
			self.affect.tick()
			self.express.tick()
			self.action.tick()
			self.loop.tick()



class DemoSystem(object):

	def __init__(self, client_type):

		# client type
		self.client_type = client_type
		if client_type == "caml" or client_type == "camr":
			self.client_type = "camera"
			self.camera_sub = client_type[-1:]
			self.stream_index = 0
			if client_type == "camr":
				self.stream_index = 1

		# config
		self.use_external_kc = False

		# pars
		self.pars = pars.CorePars()

		# resources
		self.bridge = CvBridge()

		# init ROS
		rospy.init_node(self.pars.ros.robot_name + "_client_demo_" + client_type, log_level=self.pars.ros.log_level)
		self.topic_base_name = "/" + self.pars.ros.robot_name + "/"

		# subs
		self.kc_m = miro.utils.kc_interf.kc_miro()
		self.kc_s = miro.utils.kc_interf.kc_miro()
		self.input = DemoInput()
		self.state = DemoState(self.pars)
		self.output = DemoOutput()
		self.nodes = DemoNodes(self.client_type)

		# state
		self.active_counter = 1
		self.active = False
		self.platform_flags = -1

		# monitor use of time (set timing0 to "None" to disable timing)
		self.timing = [[], [], []]
		self.timing0 = None # time.time()

		# ROS interfaces
		self.sub = []

		# select client type
		if self.client_type == "main":

			# publish priority
			self.pub_pri = [
				self.publish('core/pril', sensor_msgs.msg.Image),
				self.publish('core/prir', sensor_msgs.msg.Image),
				self.publish('core/priw', sensor_msgs.msg.Image)
				]

			# publish control outputs
			self.pub_cos = self.publish('control/cosmetic_joints', std_msgs.msg.Float32MultiArray)
			self.pub_illum = self.publish('control/illum', std_msgs.msg.UInt32MultiArray)

			# publish core states
			self.pub_affect_state = self.publish('core/affect/state', miro.msg.affect_state)
			self.pub_affect_time = self.publish('core/affect/time', std_msgs.msg.UInt32)
			self.pub_sel_prio = self.publish('core/selection/priority', std_msgs.msg.Float32MultiArray)
			self.pub_sel_inhib = self.publish('core/selection/inhibition', std_msgs.msg.Float32MultiArray)

			# reference core states output messages in output array
			self.output.affect_state = self.pub_affect_state.msg
			self.output.affect_time = self.pub_affect_time.msg
			self.output.sel_prio = self.pub_sel_prio.msg
			self.output.sel_inhib = self.pub_sel_inhib.msg

			# publish
			self.pub_flags = self.publish('control/flags', std_msgs.msg.UInt32)
			self.pub_tone = self.publish('control/tone', std_msgs.msg.UInt16MultiArray)

			# publish motor output
			if self.use_external_kc:
				self.pub_push = self.publish('core/mpg/push', miro.msg.push)
				self.pub_reset = self.publish('core/mpg/reset', std_msgs.msg.UInt32)
			else:
				self.pub_kin = self.publish('control/kinematic_joints', sensor_msgs.msg.JointState)
				self.pub_kin.msg.name = ['tilt', 'lift', 'yaw', 'pitch']
				self.pub_cmd_vel = self.publish('control/cmd_vel', geometry_msgs.msg.TwistStamped)

		# select client type
		if self.client_type == "camera":

			# publish motion detector output
			self.pub_mov = self.publish('core/mov' + self.camera_sub, sensor_msgs.msg.Image)

			# publish face detector output
			self.pub_detect_face = self.publish('core/detect_face_' + self.camera_sub, std_msgs.msg.Float32MultiArray)

			# publish ball detector output
			self.pub_detect_ball = self.publish('core/detect_ball_' + self.camera_sub, std_msgs.msg.UInt16MultiArray)

			# publish command so we can reconfigure cameras
			self.pub_command = self.publish('control/command', std_msgs.msg.String)

			# publish detection summary (debug)
			self.pub_sum = self.publish('core/sum' + self.camera_sub, sensor_msgs.msg.Image)

			# publish ball detector state (debug)
			self.pub_bal = self.publish('core/bal' + self.camera_sub, sensor_msgs.msg.Image)

		# select client type
		if self.client_type == "mics":

			# publish
			self.pub_mics = self.publish('core/audio_event', std_msgs.msg.Float32MultiArray)

		# instantiate nodes
		self.nodes.instantiate(self)

		# finalize parameters
		self.pars.finalize()

		# action final parameters
		if not self.pars.flags.DEV_RECONFIG_CAMERA_QUICK:
			self.state.reconfigure_cameras = True

		# and set up to reconfigure them on the fly
		self.trigger_filename = os.getenv("MIRO_DIR_STATE") + "/client_demo.reread"

		# select client type
		if self.client_type == "main":

			# subscribe
			self.subscribe('sensors/package', miro.msg.sensors_package, self.callback_sensors_package)
			self.subscribe('core/voice_state', miro.msg.voice_state, self.callback_voice_state)
			self.subscribe('core/detect_face_l', std_msgs.msg.Float32MultiArray, self.callback_detect_face)
			self.subscribe('core/detect_face_r', std_msgs.msg.Float32MultiArray, self.callback_detect_face)
			self.subscribe('core/detect_ball_l', std_msgs.msg.UInt16MultiArray, self.callback_detect_ball)
			self.subscribe('core/detect_ball_r', std_msgs.msg.UInt16MultiArray, self.callback_detect_ball)
			self.subscribe('core/audio_event', std_msgs.msg.Float32MultiArray, self.callback_audio_event)
			self.subscribe('core/movl', sensor_msgs.msg.Image, self.callback_movl)
			self.subscribe('core/movr', sensor_msgs.msg.Image, self.callback_movr)

		# select client type
		if self.client_type == "camera":

			# subscribe
			self.subscribe('sensors/cam' + self.camera_sub + '/compressed', sensor_msgs.msg.CompressedImage, self.callback_cam)
			self.subscribe('core/affect/state', miro.msg.affect_state, self.callback_affect_state)

		# select client type
		if self.client_type == "mics":

			# subscribe
			self.subscribe('sensors/mics', std_msgs.msg.Int16MultiArray, self.callback_mics)
			self.subscribe('core/affect/state', miro.msg.affect_state, self.callback_affect_state)

		# wait for connection before moving off
		print "waiting for connection..."
		time.sleep(1)

		# select client type
		if self.client_type == "main":

			# reset body (dev only)
			if self.use_external_kc:
				print "**** RESET BODY DEV ONLY ****"
				#self.pub_reset.msg.data = 1;
				self.pub_reset.publish()

		# set active
		self.active = True

	def subscribe(self, topic_name, data_type, callback):

		full_topic_name = self.topic_base_name + topic_name
		print "subscribing to", full_topic_name, "..."
		self.sub.append(rospy.Subscriber(full_topic_name, data_type, callback, queue_size=1, tcp_nodelay=True))

	def publish(self, topic_name, data_type):

		return DemoPub(rospy.Publisher(self.topic_base_name + topic_name, data_type, queue_size=0, tcp_nodelay=True), data_type)

	def callback_sensors_package(self, msg):

		if not self.active:
			return

		if not self.timing0 is None:
			self.timing[0].append(time.time() - self.timing0)

		# store
		self.input.sensors_package = msg

		# configure kc_s
		self.kc_s.setConfig(msg.kinematic_joints.position)

		# don't configure kc_m, it causes drift and feedback
		# effects (e.g. around movements associated with sleep)
		#self.state.motors_active = self.kc_m.setConfigIfInactive(msg.kinematic_joints.position)

		# instead, just set the active state
		self.state.motors_active = self.kc_m.isActive()

		# tick
		self.nodes.tick()

		# clear inputs
		self.input.sensors_package = None
		self.input.voice_state = None
		self.state.audio_events_for_50Hz = []

		# publish flags only if they have changed
		platform_flags = 0
		if self.state.user_touch == 0:
			platform_flags |= miro.constants.PLATFORM_D_FLAG_DISABLE_KIN_IDLE
		if self.pars.flags.BODY_ENABLE_CLIFF_REFLEX == 0:
			platform_flags |= miro.constants.PLATFORM_D_FLAG_DISABLE_CLIFF_REFLEX
		if self.pars.flags.BODY_ENABLE_TRANSLATION == 0:
			platform_flags |= miro.constants.PLATFORM_D_FLAG_DISABLE_TRANSLATION
		if self.pars.flags.BODY_ENABLE_ROTATION == 0:
			platform_flags |= miro.constants.PLATFORM_D_FLAG_DISABLE_WHEELS
		if self.platform_flags != platform_flags:
			print "publishing flags", platform_flags
			self.platform_flags = platform_flags
			self.pub_flags.msg.data = platform_flags
			self.pub_flags.publish()

		# publish
		self.pub_cos.msg.data = self.output.cosmetic_joints
		self.pub_cos.publish()

		# publish
		self.pub_illum.msg.data = self.output.illum
		self.pub_illum.publish()

		# publish core states
		self.output.affect_state.flags = 0
		if self.pars.flags.EXPRESS_THROUGH_VOICE != 0:
			self.output.affect_state.flags |= miro.constants.AFFECT_EXPRESS_THROUGH_VOICE
		if self.pars.flags.EXPRESS_THROUGH_NECK != 0:
			self.output.affect_state.flags |= miro.constants.AFFECT_EXPRESS_THROUGH_NECK
		if self.pars.flags.EXPRESS_THROUGH_WHEELS != 0:
			self.output.affect_state.flags |= miro.constants.AFFECT_EXPRESS_THROUGH_WHEELS
		self.pub_affect_state.publish()
		self.pub_affect_time.publish()
		self.pub_sel_prio.publish()
		self.pub_sel_inhib.publish()

		# publish motor output
		if self.use_external_kc:

				msg = self.pub_push.msg
				msg.pushpos = geometry_msgs.msg.Vector3(push.pos[0], push.pos[1], push.pos[2])
				msg.pushvec = geometry_msgs.msg.Vector3(push.vec[0], push.vec[1], push.vec[2])
				msg.flags = push.flags
				msg.link = push.link
				self.pub_push.publish()
		else:
			# get config & dpose from kc
			config = self.kc_m.getConfig()
			dpose = self.kc_m.getPoseChange() * miro.constants.PLATFORM_TICK_HZ

			# handle wakefulness
			w = self.state.wakefulness
			config[1] = miro.constants.LIFT_RAD_MAX + w * (config[1] - miro.constants.LIFT_RAD_MAX)

			# publish
			self.pub_kin.msg.position = config
			self.pub_kin.publish()
			self.pub_cmd_vel.msg.twist.linear.x = dpose[0]
			self.pub_cmd_vel.msg.twist.angular.z = dpose[1]
			self.pub_cmd_vel.publish()

		# clear pushes for external kc
		self.output.pushes = []

		# publish
		if self.output.tone > 0:

			# output tones are debug tones
			x = self.output.tone
			if x < 0: x = 0
			if x > 255: x = 255

			# over 250 are handled specially
			if x <= 250:
				self.output.tone = 0
			else:
				self.output.tone -= 1
				x = (x - 250) * 50

			# dispatch
			msg = self.pub_tone.msg
			msg.data = [x + 440, x, 1]
			self.pub_tone.publish()

		# tick counter
		self.state.tick += 1

		if not self.timing0 is None:
			self.timing[0].append(time.time() - self.timing0)

		# update config (from run state file)
		if os.path.isfile(self.trigger_filename):
			self.pars.finalize()
			os.remove(self.trigger_filename)

	def callback_detect_face(self, msg):

		x = msg.data
		stream_index = int(x[0])
		i = 1
		faces = []
		while i < len(x):
			faces.append(x[i:i+5])
			i += 5
		self.state.detect_face[stream_index] = faces

	def callback_detect_ball(self, msg):

		x = msg.data
		stream_index = x[0]
		if len(x) == 4:
			self.state.detect_ball[stream_index] = x[1:]
		else:
			self.state.detect_ball[stream_index] = None

	def callback_mov(self, stream_index, msg):

		if not self.active:
			return

		# store
		self.state.frame_mov[stream_index] = self.bridge.imgmsg_to_cv2(msg, "mono8")

		# tick
		updated = self.nodes.spatial.tick_camera(stream_index)

		# publish
		for i in updated:
			frame_pri = self.state.frame_pri[i]
			if not frame_pri is None:
				msg = self.bridge.cv2_to_imgmsg(frame_pri, encoding='mono8')
				self.pub_pri[i].pub.publish(msg)

	def callback_movl(self, msg):

		self.callback_mov(0, msg)

	def callback_movr(self, msg):

		self.callback_mov(1, msg)

	def callback_cam(self, msg):

		stream_index = self.stream_index

		if not self.active:
			return

		if self.active_counter == 0:
			return

		self.active_counter -= 1
		if self.active_counter == 0:
			print "going inactive..."
			return

		if not self.timing0 is None:
			self.timing[1].append(time.time() - self.timing0)

		# tick camera
		if self.nodes.decode.tick_camera(stream_index, msg):

			# tick_camera returns true to indicate the frame
			# was the wrong size and we need to reconfigure
			if self.stream_index == 0:

				# only caml sends the command to reconfigure
				print "reconfiguring cameras..."
				self.pub_command.msg.data = "frame=180w@10;jpeg=75"
				self.pub_command.publish()

			# either camera ends processing there
			return

		# tick
		self.nodes.detect_motion.tick_camera(stream_index)
		self.nodes.detect_face.tick_camera(stream_index)
		self.nodes.detect_ball.tick_camera(stream_index)

		# publish result
		frame_mov = self.state.frame_mov[stream_index]
		if not frame_mov is None:
			msg = self.bridge.cv2_to_imgmsg(frame_mov, encoding='mono8')
			self.pub_mov.pub.publish(msg)

		# publish result
		faces = self.state.detect_face[stream_index]
		msg = self.pub_detect_face.msg
		msg.data = []
		msg.data.append(stream_index)
		for face in faces:
			for f in face:
				msg.data.append(f)
		self.pub_detect_face.publish()

		# publish result
		ball = self.state.detect_ball[stream_index]
		msg = self.pub_detect_ball.msg
		msg.data = []
		msg.data.append(stream_index)
		if not ball is None:
			for f in ball:
				msg.data.append(f)
		self.pub_detect_ball.publish()

		# dev
		if self.pars.flags.DEV_SEND_DEBUG_TOPICS:

			# publish
			frame_sum = copy.copy(self.state.frame_raw[stream_index])
			if not frame_sum is None:
				ball = self.state.detect_ball[stream_index]
				if not ball is None:
					self.state.detect_ball[stream_index] = None
					cv2.circle(frame_sum, (ball[0], ball[1]), ball[2], (0, 255, 0), 1)
				faces = self.state.detect_face[stream_index]
				if not faces is None:
					for face in faces:
						(x1, y1, xw, yw, conf) = face
						x1 = int(x1)
						y1 = int(y1)
						x2 = int(x1 + xw)
						y2 = int(y1 + yw)
						cv2.rectangle(frame_sum, (x1, y1), (x2, y2), (0, 255, 0), 1)
				msg = self.bridge.cv2_to_imgmsg(frame_sum, encoding='bgr8')
				self.pub_sum.pub.publish(msg)

			# publish
			frame_bal = self.state.frame_bal[stream_index]
			if not frame_bal is None:
				msg = self.bridge.cv2_to_imgmsg(frame_bal, encoding='mono8')
				self.pub_bal.pub.publish(msg)

		if not self.timing0 is None:
			self.timing[1].append(time.time() - self.timing0)

	def callback_voice_state(self, msg):

		if not self.active:
			return

		self.input.voice_state = msg

	def callback_mics(self, msg):

		if not self.active:
			return

		if self.active_counter == 0:
			return

		self.active_counter -= 1
		if self.active_counter == 0:
			print "going inactive..."
			return

		if not self.timing0 is None:
			self.timing[2].append(time.time() - self.timing0)

		if len(msg.data) != 2000:
			print "bad data size in mics message"
			return

		self.input.mics = msg

		event = self.nodes.detect_audio.tick_mics()

		if not event is None:
			self.pub_mics.msg.data = [event.azim, event.elev, event.level]
			self.pub_mics.publish()

		if not self.timing0 is None:
			self.timing[2].append(time.time() - self.timing0)

	def callback_audio_event(self, msg):

		q = DetectAudioEvent(msg.data)
		self.state.audio_events_for_spatial.append(q)
		self.state.audio_events_for_50Hz.append(q)

	def callback_affect_state(self, msg):

		if self.active_counter == 0:
			print "going active..."

		# this is used only to cause the background nodes to go active
		# when the foreground node "main" starts running, and vice versa
		self.active_counter = 50

	def loop(self):

		# main loop
		while not rospy.core.is_shutdown() and self.state.keep_running:

			# sleepy time
			time.sleep(0.1)

			# check run file
			if not run_file is None:
				if not os.path.isfile(run_file):
					print "run file disappeared, exiting..."
					break

		# set inactive
		self.active = False

		# timing
		if not self.timing0 is None:
			print "\n\n\n"
			np.set_printoptions(precision=6, linewidth=1000000)
			for i in range(3):
				tt = self.timing[i]
				print np.array(tt)



################ ARGS ################

# when running on board, the single argument may indicate the
# run file; if so, we should exit if this file disappears
run_file = None
if len(sys.argv) > 1:
	run_file = sys.argv[1]

	# special run_file "-" means do not use
	if run_file == "-":
		run_file = None

# a second argument determines which client to run
client_type = "main"
if len(sys.argv) > 2:
	client_type = sys.argv[2]
	if not (client_type == "caml" or client_type == "camr" or client_type == "mics"):
		print "unrecognised client type"
		exit()



################ MAIN ################

# instantiate
demo = DemoSystem(client_type)

# execute
demo.loop()




