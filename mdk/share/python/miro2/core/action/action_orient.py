#	@section COPYRIGHT
#	Copyright (C) 2019 Consequential Robotics Ltd
#	
#	@section AUTHOR
#	Consequential Robotics http://consequentialrobotics.com
#	
#	@section LICENSE
#	For a full copy of the license agreement, see LICENSE in the
#	MDK root directory.
#	
#	Subject to the terms of this Agreement, Consequential
#	Robotics grants to you a limited, non-exclusive, non-
#	transferable license, without right to sub-license, to use
#	MIRO Developer Kit in accordance with this Agreement and any
#	other written agreement with Consequential Robotics.
#	Consequential Robotics does not transfer the title of MIRO
#	Developer Kit to you; the license granted to you is not a
#	sale. This agreement is a binding legal agreement between
#	Consequential Robotics and the purchasers or users of MIRO
#	Developer Kit.
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

	def compute_priority(self):

		return self.input.priority_peak.height * self.pars.action.orient_base_prio

	def event_start(self):

		self.appetitive_response(self.pars.action.orient_appetitive_commitment)
		self.debug_event_start()

		if self.pars.flags.DEV_DEBUG_ORIENT_START:
			self.system_output.tone = 240

	def start(self):

		# get current gaze target in HEAD
		self.gaze_HEAD = miro.utils.kc_interf.kc_view_to_HEAD(
				0.0,
				self.input.gaze_elevation,
				self.pars.action.orient_gaze_target_radius
				)

		# get current gaze target in WORLD
		self.gaze_i_WORLD = self.kc.changeFrameAbs(
				miro.constants.LINK_HEAD,
				miro.constants.LINK_WORLD,
				miro.utils.kc_interf.kc_view_to_HEAD(
					0.0,
					self.input.gaze_elevation,
					self.pars.action.orient_gaze_target_radius
					)
				)
		#print "self.gaze_i_WORLD", self.gaze_i_WORLD

		# get desired gaze target in WORLD
		self.gaze_f_WORLD = self.kc.changeFrameAbs(
				miro.constants.LINK_HEAD,
				miro.constants.LINK_WORLD,
				miro.utils.kc_interf.kc_view_to_HEAD(
					self.input.priority_peak.azim,
					self.input.priority_peak.elev,
					self.pars.action.orient_gaze_target_radius
					)
				)
		#print "self.gaze_f_WORLD", self.gaze_f_WORLD

		# get change in gaze target across movement
		self.dgaze_WORLD = self.gaze_f_WORLD - self.gaze_i_WORLD

		# decide pattern rate / time
		elev = self.input.priority_peak.elev - self.input.gaze_elevation
		rad = np.sqrt(np.square(self.input.priority_peak.azim) + np.square(elev))
		sec_ideal = rad * self.pars.action.orient_speed_sec_per_rad
		steps_ideal = int(sec_ideal * self.pars.timing.tick_hz)
		steps = np.clip(steps_ideal, self.pars.action.orient_min_steps, self.pars.action.orient_max_steps)

		# start action clock
		self.clock.start(steps)

	def service(self):

		# read clock
		x = self.clock.cosine_profile()
		self.clock.advance(True)

		# compute interim gaze target along a straight line (not quite an arc, but no matter...)
		gaze_x_WORLD = self.gaze_i_WORLD + x * self.dgaze_WORLD
		#print "gaze_x_WORLD", gaze_x_WORLD

		# transform into HEAD for actioning as a push
		gaze_x_HEAD = self.kc.changeFrameAbs(
				miro.constants.LINK_WORLD,
				miro.constants.LINK_HEAD,
				gaze_x_WORLD
				)
		#print "gaze_x_HEAD", gaze_x_HEAD

		# correct range to gaze target range so we turn rather than shimmy
		# (we won't shimmy anyway with NO_TRANSLATION, but still...)
		gaze_x_HEAD *= self.pars.action.orient_gaze_target_radius / np.linalg.norm(gaze_x_HEAD)

		# apply push
		push = miro.utils.kc.KinematicPush()
		push.link = miro.constants.LINK_HEAD
		push.flags = 0 \
				| miro.constants.PUSH_FLAG_IMPULSE \
				| miro.constants.PUSH_FLAG_NO_TRANSLATION
		push.pos = self.gaze_HEAD
		push.vec = gaze_x_HEAD - self.gaze_HEAD
		self.apply_push(push)

		#print push.vec

