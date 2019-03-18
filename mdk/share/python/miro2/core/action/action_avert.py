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
#	Action Avert
#	MIRO turns away from stimulus


import rospy
import copy
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

		# modulate for dev
		if self.pars.flags.DEV_ORIENT_ONLY:
			priority = 0.0

		# ok
		return priority

	def start(self):

		"""
		# compute start point for fovea in WORLD
		self.fovea_i_WORLD = self.kc.changeFrameAbs(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, self.fovea_HEAD)

		# compute rotate point for movement in WORLD
		self.rotate_cen_WORLD = self.kc.changeFrameAbs(miro.constants.LINK_FOOT, miro.constants.LINK_WORLD, np.array([-0.3, 0.0, 0.0]))
		self.rotate_cen_WORLD[2] = self.fovea_i_WORLD[2]

		# compute start point as relative vector
		self.fovea_i_REL = self.fovea_i_WORLD - self.rotate_cen_WORLD
		"""

		# compute pattern time
		secs_ideal = 2.0
		steps_ideal = int(secs_ideal * self.pars.timing.tick_hz)
		steps_constrained = np.clip(steps_ideal,
					self.pars.action.approach_min_steps,
					self.pars.action.approach_max_steps
					)

		# mark pattern direction
		if self.input.cliff[1] > self.input.cliff[0]:
			dtheta = np.pi
		else:
			dtheta = -np.pi
		#dtheta = np.pi

		# scale pattern direction by pattern distance
		self.dfov = dtheta * 0.13

		# start pattern
		self.x_bak = 0.0
		self.clock.start(steps_constrained)

		# debug
		if self.debug:
			print "rotate_cen_WORLD", self.rotate_cen_WORLD
			print "fovea_i_REL", self.fovea_i_REL
			print "pattern time", secs_ideal, steps_ideal, steps_constrained

	def service(self):

		# read clock
		x = self.clock.cosine_profile()
		self.clock.advance(True)

		# get step
		dx = x - self.x_bak
		self.x_bak = x

		"""
		# compute theta
		theta = x * self.dtheta

		# compute rots
		c = np.cos(theta)
		s = np.sin(theta)

		# compute interim target
		x = copy.copy(self.fovea_i_REL)
		y = copy.copy(self.fovea_i_REL)
		y[0] = c * x[0] - s * x[1]
		y[1] = s * x[0] + c * x[1]
		fovea_x_WORLD = self.rotate_cen_WORLD + y

		# add a bit of backwards motion
		q = self.clock.linear_profile()
		#fovea_x_WORLD -= q * x
		#print q, theta, c, s

		# transform interim target into HEAD for actioning
		fovea_x_HEAD = self.kc.changeFrameAbs(miro.constants.LINK_WORLD, miro.constants.LINK_HEAD, fovea_x_WORLD)

		# apply push
		self.apply_push_fovea(fovea_x_HEAD - self.fovea_HEAD)

		fovea_now_WORLD = \
			self.kc.changeFrameAbs(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, self.fovea_HEAD)
		print fovea_x_WORLD, fovea_now_WORLD
		"""

		# compute vector
		vec = np.array([0.0, self.dfov * dx, 0.0])
		self.apply_push_fovea(vec)
