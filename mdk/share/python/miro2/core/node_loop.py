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

import numpy as np
import time

import node
import miro2 as miro



class NodeLoop(node.Node):

	def __init__(self, sys):

		node.Node.__init__(self, sys, "loop")

		# state
		self.cos_bak = np.array([0.0] * 6)
		self.fovea_HEAD = miro.utils.get("LOC_SONAR_FOVEA_HEAD")
		self.fovea_WORLD_bak = self.kc_m.changeFrameAbs(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, self.fovea_HEAD)
		self.in_blink_count = 0
		self.in_cos_body_count = 0
		self.in_cos_head_count = 0
		self.in_motion_count = 0
		self.in_vocalising_count = 0

	def tick(self):

		# measure movement of cosmetic joints

		cos = self.output.cosmetic_joints

		if np.max(cos[2:4]) == 1.0:
			self.in_blink_count = self.pars.platform.n_blink_settle
		elif self.in_blink_count > 0:
			self.in_blink_count -= 1

		self.state.in_blink = float(self.in_blink_count) / self.pars.platform.n_blink_settle

		cos = np.round(cos * 100)
		dcos = cos - self.cos_bak

		if np.any(dcos[0:2]):
			self.in_cos_body_count = self.pars.platform.n_body_cos_settle
		elif self.in_cos_body_count > 0:
			self.in_cos_body_count -= 1

		self.state.in_cos_body = float(self.in_cos_body_count) / self.pars.platform.n_body_cos_settle

		if np.any(dcos[2:6]):
			self.in_cos_head_count = self.pars.platform.n_head_cos_settle
		elif self.in_cos_head_count > 0:
			self.in_cos_head_count -= 1

		self.state.in_cos_head = float(self.in_cos_head_count) / self.pars.platform.n_head_cos_settle

		self.cos_bak = cos

		# measure movement of fovea
		#
		# measure movement of fovea in kc_m - this is necessary
		# because speed in kc_s is non-zero when we are actively
		# moving but also when the plant is moved by external
		# forces - or, even, when the sensors jitter, which can
		# be unhelpful if what we actually want to measure is
		# "are the motors driving and making (e.g.) noise?".
		#
		# in general, it might be useful to measure both, but
		# currently we only measure that of kc_m.
		fovea_WORLD = self.kc_m.changeFrameAbs(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, self.fovea_HEAD)
		dfovea = fovea_WORLD - self.fovea_WORLD_bak
		self.fovea_WORLD_bak = fovea_WORLD
		self.state.fovea_speed = np.linalg.norm(dfovea) * miro.constants.PLATFORM_TICK_HZ

		# and we actually base our "in_motion" flag not on
		# this, but on whether the "motors_active" flag is set,
		# which reflects whether the kc_m is being pushed at
		# the current moment.
		if self.state.motors_active:
			self.in_motion_count = self.pars.platform.n_mov_settle
		elif self.in_motion_count > 0:
			self.in_motion_count -= 1
		self.state.in_motion = float(self.in_motion_count) / self.pars.platform.n_mov_settle

		# measure vocalisation activity
		if self.input.voice_state is None:
			# probably voice node is not loaded
			self.in_vocalising_count = 0
		else:
			if self.input.voice_state.vocalising:
				self.in_vocalising_count = self.pars.platform.n_vocalising_settle
			elif self.in_vocalising_count > 0:
				self.in_vocalising_count -= 1
		self.state.in_vocalising = float(self.in_vocalising_count) / self.pars.platform.n_vocalising_settle

		# measure any activity that makes noise
		#
		# TODO: measure separate constants for time to settle from
		# these various stimuli in generating (a) activity on the
		# IMU and (b) sound that is detected by the mics; they are
		# different, and we're currently treating them as the same
		# which means we will miss some signals we needn't miss.
		self.state.in_making_noise = np.array([self.state.in_motion, self.state.in_vocalising, self.state.in_cos_head]).max()



