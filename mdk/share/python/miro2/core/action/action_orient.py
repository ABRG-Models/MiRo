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
#
#	Action Orient
#	action which orients MiRo facing a stimulus


import rospy
import copy
import numpy as np

import miro2 as miro

from action_types import ActionTemplate



class ActionOrient(ActionTemplate):

	def finalize(self):

		# parameters
		self.name = "orient"

		if self.pars.dev.DEBUG_WRITE_TRACES:
			with open('/tmp/orient', 'w') as file:
				file.write("")

	def compute_priority(self):

		return self.move_softsat(self.input.priority_peak.height)

	def event_start(self):

		self.appetitive_response(self.pars.action.orient_appetitive_commitment)
		self.debug_event_start()

		if self.pars.dev.DEBUG_ORIENT_START:
			self.system_output.tone = 240

		if self.pars.dev.DEBUG_WRITE_TRACES:
			with open('/tmp/orient', 'a') as file:
				s_peak = str(self.input.priority_peak.azim) + " " + str(self.input.priority_peak.elev)
				file.write(str(self.system_state.tick) + " " + s_peak + "\n")

	def start(self):

		# first, we determine where is the object to which we intend
		# to orient, in WORLD frame. this won't change for the duration
		# of the action.

		# determine desired final gaze target in SENSOR
		self.gaze_target_f_SENSOR = miro.utils.kc_interf.kc_viewline_to_position(
				self.input.priority_peak.azim,
				self.input.priority_peak.elev,
				self.input.priority_peak.range
				)

		# transform into HEAD
		if self.input.priority_peak.stream_index == 0:
			gaze_target_f_HEAD = self.gaze_target_f_SENSOR + self.pars.camera.location[0]
		if self.input.priority_peak.stream_index == 1:
			gaze_target_f_HEAD = self.gaze_target_f_SENSOR + self.pars.camera.location[1]
		else:
			gaze_target_f_HEAD = self.gaze_target_f_SENSOR

		# transform into WORLD
		self.gaze_target_f_WORLD = self.kc.changeFrameAbs(
				miro.constants.LINK_HEAD,
				miro.constants.LINK_WORLD,
				gaze_target_f_HEAD
				)

		# we can do orienting of gaze either in the space of the stream
		# sensor (caml, camr, mics) or in HEAD. there are pros and cons
		# to each.
		#
		# if we do it in HEAD, there is no "sensor bias" - the whole head
		# will be looking straight at you afterwards; if we do it in SENSOR
		# then we'll end up with just one eye looking at us (usually).

		# get current gaze target at the same range as that, in a form
		# that is suitable for use in either HEAD or SENSOR
		self.gaze_target_i_HEAD_OR_SENSOR = miro.utils.kc_interf.kc_viewline_to_position(
				0.0,
				self.input.gaze_elevation,
				self.input.priority_peak.range
				)

		# orient in HEAD
		if self.pars.action.orient_in_HEAD:

			# define gaze target in HEAD as canonical gaze target
			self.gaze_target_i_HEAD = self.gaze_target_i_HEAD_OR_SENSOR

		# orient in SENSOR
		else:

			# define gaze target in HEAD as canonical gaze target
			# transformed from SENSOR into HEAD
			if self.input.priority_peak.stream_index == 0:
				self.gaze_target_i_HEAD = self.gaze_target_i_HEAD_OR_SENSOR + self.pars.camera.location[0]
			if self.input.priority_peak.stream_index == 1:
				self.gaze_target_i_HEAD = self.gaze_target_i_HEAD_OR_SENSOR + self.pars.camera.location[1]
			if self.input.priority_peak.stream_index == 2:
				self.gaze_target_i_HEAD = self.gaze_target_i_HEAD_OR_SENSOR

		# get current gaze target in WORLD
		self.gaze_target_i_WORLD = self.kc.changeFrameAbs(
				miro.constants.LINK_HEAD,
				miro.constants.LINK_WORLD,
				self.gaze_target_i_HEAD
				)

		# debug
		if self.pars.dev.DEBUG_ORIENTS:

			print "\n\nORIENT START!\n\n"

			"""
			# get desired gaze target in WORLD
			self.gaze_target_f_WORLD = self.kc.changeFrameAbs(
					miro.constants.LINK_HEAD,
					miro.constants.LINK_WORLD,
					miro.utils.kc_interf.kc_viewline_to_position(
						0.4,
						self.input.gaze_elevation + 0.2,
						self.pars.action.orient_gaze_target_radius
						)
					)

			print "---- gazes ----"
			print self.gaze_target_i_WORLD
			print self.gaze_target_f_WORLD
			"""

		# get change in gaze target across movement
		self.dgaze_target_WORLD = self.gaze_target_f_WORLD - self.gaze_target_i_WORLD

		# decide pattern rate / time
		elev = self.input.priority_peak.elev - self.input.gaze_elevation
		rad = np.sqrt(np.square(self.input.priority_peak.azim) + np.square(elev))
		sec_ideal = rad * self.pars.action.orient_speed_sec_per_rad
		steps_ideal = int(sec_ideal * self.pars.timing.tick_hz)
		steps = np.clip(steps_ideal, self.pars.action.orient_min_steps, self.pars.action.orient_max_steps)

		# start action clock
		self.clock.start(steps)

		# mark actioned
		self.input.priority_peak.actioned = 1

	def service(self):

		# read clock
		x = self.clock.cosine_profile()
		self.clock.advance(True)

		# compute desired gaze target along a straight line (not quite an arc, but no matter...)
		gaze_target_WORLD_cmd = self.gaze_target_i_WORLD + x * self.dgaze_target_WORLD

		# debug how WORLD gaze is intended to, and actually does, change
		if self.pars.dev.DEBUG_ORIENTS:
			gaze_target_WORLD = self.kc.changeFrameAbs(
					miro.constants.LINK_HEAD,
					miro.constants.LINK_WORLD,
					self.gaze_target_i_HEAD
					)
			print gaze_target_WORLD_cmd, gaze_target_WORLD

		# transform into HEAD for actioning as a push
		gaze_x_HEAD = self.kc.changeFrameAbs(
				miro.constants.LINK_WORLD,
				miro.constants.LINK_HEAD,
				gaze_target_WORLD_cmd
				)

		# we compute a trajectory in WORLD that is a straight line, which
		# in principle will cause us to shimmy backwards rather than simply
		# turning our head. in practice, this is prevented by using the flag
		# PUSH_FLAG_IMPULSE anyway, but for belt and braces we reconstruct
		# the arc, here, approximately, unless flagged off.
		if self.pars.action.orient_follow_arc:
			gaze_x_HEAD *= np.linalg.norm(self.gaze_target_f_SENSOR) / np.linalg.norm(gaze_x_HEAD)

		# prepare push
		push = miro.utils.kc.KinematicPush()
		push.link = miro.constants.LINK_HEAD
		push.flags = 0 \
				| miro.constants.PUSH_FLAG_IMPULSE \
				| miro.constants.PUSH_FLAG_NO_TRANSLATION
		push.pos = self.gaze_target_i_HEAD
		push.vec = gaze_x_HEAD - self.gaze_target_i_HEAD

		# debug
		#print x, np.linalg.norm(push.vec)

		# debug (do not apply push)
		#if self.pars.dev.DEBUG_ORIENTS:
		#	if self.count != 3:
		#		return

		# apply push
		self.apply_push(push)
