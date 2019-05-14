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
#	Action Retreat
#	action where Miro will retreat a stimulus

import copy
import numpy as np
import miro2 as miro

from action_types import ActionTemplate



class ActionRetreat(ActionTemplate):

	def finalize(self):

		# parameters
		self.name = "retreat"
		self.retreatable = True
		self.move_away = True

		# state
		self.n_bid = 0

	def compute_priority(self):

		# if retreat distance is zero, never bid
		if self.pars.action.retreat_distance_m == 0:
			return 0.0

		# if nothing to key off, never bid
		if self.parent.retreatable_push is None:
			return 0.0

		# if halting
		if self.system_state.halting:

			# start bidding
			self.n_bid = 5

		# if bidding
		if self.n_bid:

			# bid
			self.n_bid -= 1
			#print self.n_bid
			return self.pars.action.priority_medium

		# otherwise
		return 0.0

	def start(self):

		# get push
		push = self.parent.retreatable_push
		self.parent.retreatable_push = None

		# if gone, cannot start
		if push is None:
			return

		# Fixed retreat distance
		distance = self.pars.action.retreat_distance_m
		distance *= (1.0 + self.pars.action.retreat_rand_gain * np.random.uniform())

		# Fixed retreat spead
		speed = self.pars.action.retreat_speed_mps
		speed *= (1.0 + self.pars.action.retreat_rand_gain * np.random.uniform())

		# pattern length
		time = distance / speed
		steps_total = int(time * self.pars.timing.tick_hz)

		# if None, we should not have come
		if push is None:
			miro.utils.warning("retreat without retreatable push")
			return

		# get previous foveal vector
		pushvec_HEAD = copy.copy(push.vec)

		# reverse its direction
		pushvec_HEAD *= -1.0

		# transform retreat vector into world
		pushvec_WORLD = self.kc.changeFrameRel(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, pushvec_HEAD)

		# zero out vertical
		pushvec_WORLD[2] = 0.0

		# scale by retreat distance
		pushvec_WORLD *= (1.0 / np.linalg.norm(pushvec_WORLD))
		pushvec_WORLD *= distance

		# retreat vector
		self.dfovea_WORLD = pushvec_WORLD

		# start point for fovea in WORLD
		self.fovea_i_WORLD = self.kc.changeFrameAbs(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, self.fovea_HEAD)

		# end point for fovea in WORLD
		fovea_f_WORLD = self.fovea_i_WORLD + self.dfovea_WORLD

		# fix end point z value
		fovea_f_WORLD[2] = (self.pars.geom.reachable_z_min + self.pars.geom.reachable_z_max) * 0.5

		# recompute dfovea
		self.dfovea_WORLD = fovea_f_WORLD - self.fovea_i_WORLD

		# start pattern
		self.clock.start(steps_total)

		# clear bid
		self.n_bid = 0

	def service(self):

		# read clock
		x = self.clock.cosine_profile()
		self.clock.advance(True)

		# compute interim target
		fovea_x_WORLD = self.fovea_i_WORLD + x * self.dfovea_WORLD

		# transform into head
		fovea_x_HEAD = self.kc.changeFrameAbs(miro.constants.LINK_WORLD, miro.constants.LINK_HEAD, fovea_x_WORLD)

		# compute and apply push
		self.apply_push_fovea(fovea_x_HEAD - self.fovea_HEAD)



