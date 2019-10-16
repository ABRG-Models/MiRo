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
#	Action Avert
#	MIRO turns away from stimulus


import rospy
import copy
import random
import numpy as np
import miro2 as miro

from action_types import ActionTemplate



class ActionAvert(ActionTemplate):

	def finalize(self):

		# parameters
		self.name = "avert"
		self.retreatable = True
		self.move_away = True

	def compute_priority(self):

		# extract variables
		valence = self.input.emotion.valence
		arousal = self.input.emotion.arousal
		height = self.input.priority_peak.height
		size_norm = self.input.priority_peak.size_norm
		fixation = self.input.fixation

		# extract pars
		base_prio = self.pars.action.avert_base_prio

		# compute priority
		priority = base_prio

		# modulate by cliff
		priority = self.modulate_priority_to(priority,
				self.pars.action.priority_high,
				1.0 - self.input.conf_surf,
				+1)

		# ok
		return priority

	def start(self):

		# get pattern direction from cliff sensor asymmetry
		if self.input.cliff[1] > self.input.cliff[0]:
			dtheta = -1.0
		elif self.input.cliff[1] < self.input.cliff[0]:
			dtheta = 1.0
		else:
			# if they are both the same, either direction is good, and it
			# makes "circling behaviour" less weird if we randomize the
			# direction it goes in. an alternative is to use the current
			# head yaw direction to control it, but it gets repetitive.
			if True:
				if random.randint(0,1) == 0:
					dtheta = 1.0
				else:
					dtheta = -1.0
			else:
				if config[2] >= 0.0:
					dtheta = 1.0
				else:
					dtheta = -1.0

		# compute movement parameters
		q = 1 + self.pars.action.avert_variability * np.random.normal()
		x = -self.pars.action.avert_retreat_distance * q
		q = 1 + self.pars.action.avert_variability * np.random.normal()
		y = dtheta * self.pars.action.avert_turn_distance * q
		#print "\nAVERT", x, y, "\n"

		# choose algorithm
		if self.pars.action.avert_algorithm == 'fovea':

			""" OLD ALGORITHM, nice looking but difficult to get a reliable retreat out of it... """

			# ignore current head position in following computations
			config_current = self.kc.getConfig()
			config_init = self.kc.getConfigInit()
			self.kc.setConfig(config_init)

			# fovea now (with head at calibration position)
			self.fovea_i_WORLD = self.kc.changeFrameAbs(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, self.fovea_HEAD)

			# restore original config
			self.kc.setConfig(config_current)

			# generate fovea target
			xyzf_FOOT = np.array([x, y, self.fovea_i_WORLD[2]])
			self.fovea_f_WORLD = self.kc.changeFrameAbs(miro.constants.LINK_FOOT, miro.constants.LINK_WORLD, xyzf_FOOT)

			# add retreat boost - this causes us to override the pattern in
			# the early stages to force a retreat movement, which makes it
			# easier to control coming away from a cliff fairly safely
			self.retreat_boost = 1.0

			# get distance to cover
			d = np.linalg.norm(self.fovea_f_WORLD - self.fovea_i_WORLD)

		# choose algorithm
		if self.pars.action.avert_algorithm == 'body':

			""" NEW ALGORITHM: prosaic, will do for now """

			# get position now
			fovea_BODY = copy.copy(self.fovea_BODY)
			self.fovea_BODY_i_WORLD = self.kc.changeFrameAbs(miro.constants.LINK_BODY, miro.constants.LINK_WORLD, fovea_BODY)

			# get position after move
			fovea_BODY += np.array([x, y, 0.0])
			self.fovea_BODY_f_WORLD = self.kc.changeFrameAbs(miro.constants.LINK_BODY, miro.constants.LINK_WORLD, fovea_BODY)

			"""
			# get current gaze target in WORLD
			self.gaze_target_i_WORLD = self.kc.changeFrameAbs(
					miro.constants.LINK_HEAD,
					miro.constants.LINK_WORLD,
					self.gaze_target_i_HEAD_OR_SENSOR
					)

			# get final gaze target (direction only) in WORLD
			fovea_BODY += np.array([x * 100.0, y * 100.0, 0.0])
			self.gaze_target_f_WORLD = self.kc.changeFrameAbs(miro.constants.LINK_BODY, miro.constants.LINK_WORLD, fovea_BODY)
			"""

			# get distance to cover
			d = np.linalg.norm(self.fovea_BODY_f_WORLD - self.fovea_BODY_i_WORLD)

		""" COMMON TO BOTH ALGORITHMS """

		# compute pattern time
		secs_ideal = d / self.pars.action.avert_mean_speed
		steps_ideal = int(secs_ideal * self.pars.timing.tick_hz)
		steps_constrained = np.clip(steps_ideal,
					self.pars.action.avert_min_steps,
					self.pars.action.avert_max_steps
					)

		# start pattern
		self.clock.start(steps_constrained)

	def service(self):

		# read clock
		x = self.clock.cosine_profile()
		self.clock.advance(True)

		# choose algorithm
		if self.pars.action.avert_algorithm == 'fovea':

			# get target
			xyz_WORLD = self.fovea_i_WORLD + x * (self.fovea_f_WORLD - self.fovea_i_WORLD)

			# in HEAD
			xyz_HEAD = self.kc.changeFrameAbs(miro.constants.LINK_WORLD, miro.constants.LINK_HEAD, xyz_WORLD)

			# compute vector
			dfov_HEAD = xyz_HEAD - self.fovea_HEAD

			# if using retreat boost
			if self.pars.action.avert_retreat_boost > 0:

				# NB: this doesn't work yet, it's a work in progress; I
				# think the open loop control of body is conflicting with
				# the closed loop control of fovea...

				# apply a firm backwards push at the start
				x = -self.pars.action.avert_retreat_boost * self.retreat_boost
				self.apply_push_body(np.array([x, 0.0, 0.0]), flags=miro.constants.PUSH_FLAG_VELOCITY)

				# but move over to respecting the move itself as time goes by
				self.apply_push_fovea(dfov_HEAD * (1.0 - self.retreat_boost))

				# decay retreat boost
				self.retreat_boost *= self.pars.action.avert_retreat_boost_lambda

			# if not using retreat boost
			else:

				# just apply push to HEAD
				self.apply_push_fovea(dfov_HEAD)

		# choose algorithm
		if self.pars.action.avert_algorithm == 'body':

			# get target
			xyz_WORLD = self.fovea_BODY_i_WORLD + x * (self.fovea_BODY_f_WORLD - self.fovea_BODY_i_WORLD)

			# in BODY, HEAD
			xyz_BODY = self.kc.changeFrameAbs(miro.constants.LINK_WORLD, miro.constants.LINK_BODY, xyz_WORLD)
			xyz_HEAD = self.kc.changeFrameAbs(miro.constants.LINK_BODY, miro.constants.LINK_HEAD, xyz_BODY)

			# compute vector
			dfov_BODY = xyz_BODY - self.fovea_BODY

			# do push
			self.apply_push_body(dfov_BODY)

			""" do this later...

			# compute desired gaze target along a straight line (not quite an arc, but no matter...)
			gaze_x_WORLD = self.gaze_target_i_WORLD + x * (self.gaze_target_f_WORLD - self.gaze_target_i_WORLD)

			# transform into HEAD for actioning as a push
			gaze_x_HEAD = self.kc.changeFrameAbs(
					miro.constants.LINK_WORLD,
					miro.constants.LINK_HEAD,
					gaze_x_WORLD
					)

			# correct range
			gaze_x_HEAD *= np.linalg.norm(self.gaze_target_f_SENSOR) / np.linalg.norm(gaze_x_HEAD)

			# move yaw to point at distant target
			#target = self.kc.changeFrameAbs(miro.constants.LINK_WORLD, miro.constants.LINK_HEAD, self.fovea_BODY_q_WORLD)
			#f = miro.constants.PUSH_FLAG_IMPULSE | miro.constants.PUSH_FLAG_NO_ROTATION
			#self.apply_push_fovea(np.array([0.0, -y, 0.0]), flags=f)
			"""

	def event_stop(self):

		"""
		x1 = self.kc.changeFrameAbs(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, self.fovea_HEAD)
		x2 = self.fovea_BODY_f_WORLD
		print "stop", x1, x2

		self.stop_client()
		"""
		pass
