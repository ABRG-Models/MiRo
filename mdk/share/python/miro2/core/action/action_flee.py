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
#	Action Flee
#	MIRO flees from stimulus - similar to retreat but without the checks


import rospy
import copy
import numpy as np
import miro2 as miro

from action_types import ActionTemplate



class ActionFlee(ActionTemplate):

	def finalize(self):

		# parameters
		self.name = "flee"
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
		base_prio = self.pars.action.flee_base_prio
		arousal_gain = self.pars.action.flee_arousal_gain
		valence_gain = self.pars.action.flee_valence_gain
		size_gain = self.pars.action.flee_size_gain
		fixation_gain = self.pars.action.flee_fixation_gain

		# compute priority
		priority = height * (base_prio
				- valence_gain * (valence - 0.5)
				+ arousal_gain * (arousal - 0.5)
				+ fixation_gain * (fixation - 0.5)
				+ size_gain * size_norm)

		# modulate by cliff
		priority = self.modulate_priority_to(priority,
				self.pars.action.priority_medium,
				1.0 - self.input.conf_surf,
				+1)

		# modulate for dev
		if self.pars.flags.DEV_ORIENT_ONLY:
			priority = 0.0

		# ok
		return priority

	def start(self):

		# compute start point for fovea in WORLD
		self.fovea_i_WORLD = self.kc.changeFrameAbs(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, self.fovea_HEAD)

		# flee distance
		flee_dist = self.pars.action.range_estimate_max - self.input.priority_peak.range # flee further if it is closer

		# compute end point for fovea in WORLD
		fovea_f_WORLD = self.kc.changeFrameAbs(
				miro.constants.LINK_HEAD,
				miro.constants.LINK_WORLD,
				miro.utils.kc_interf.kc_view_to_HEAD(
					self.input.priority_peak.azim + np.pi, # flee in opposite direction to priority peak
					self.input.priority_peak.elev,
					flee_dist
					)
				)

		# fix end point z value
		fovea_f_WORLD[2] = (self.pars.geom.reachable_z_min + self.pars.geom.reachable_z_max) * 0.5

		# compute total movement fovea will make in world
		self.dfovea_WORLD = fovea_f_WORLD - self.fovea_i_WORLD

		# compute pattern time
		total_dist = np.linalg.norm(self.dfovea_WORLD)
		secs_ideal = total_dist * self.pars.action.approach_speed_spm
		steps_ideal = int(secs_ideal * self.pars.timing.tick_hz)
		steps_constrained = np.clip(steps_ideal,
					self.pars.action.approach_min_steps,
					self.pars.action.approach_max_steps
					)

		# start pattern
		self.clock.start(steps_constrained)

		# debug
		if self.debug:
			print "fovea_i_WORLD", self.fovea_i_WORLD
			print "fovea_f_WORLD", fovea_f_WORLD
			print "pattern time", total_dist, secs_ideal, steps_ideal, steps_constrained

	def service(self):

		# read clock
		x = self.clock.cosine_profile()
		self.clock.advance(True)

		# compute an interim target along a straight trajectory
		fovea_x_WORLD = x * self.dfovea_WORLD + self.fovea_i_WORLD

		# transform interim target into HEAD for actioning
		fovea_x_HEAD = self.kc.changeFrameAbs(miro.constants.LINK_WORLD, miro.constants.LINK_HEAD, fovea_x_WORLD)

		# apply push
		self.apply_push_fovea(fovea_x_HEAD - self.fovea_HEAD)


