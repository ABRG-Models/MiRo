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

import copy
import numpy as np

import miro2 as miro

import signals



class ActionTemplate(object):

	def __init__(self, parent):

		# store references to objects shared between actions
		self.parent = parent
		self.pars = parent.pars
		self.kc = parent.kc_m
		self.input = parent.action_input
		self.system_state = parent.state
		self.system_output = parent.output

		# default action parameters
		self.debug = False
		self.retreatable = False # retreatable actions can evoke action retreat
		self.move_away = False # used to implement ENABLE_MOVE_AWAY flag
		self.ongoing_priority = self.pars.action.priority_medium
		self.modulate_by_wakefulness = True

		# initialise state and clock objects private to this action
		self.interface = ActionInterface()
		self.clock = ActionClock()

		# miscellaneous state
		self.name = "unnamed"
		self.fovea_HEAD = miro.utils.get("LOC_SONAR_FOVEA_HEAD")

		# call finalize (derived class must implement this)
		self.finalize()

	def modulate_priority_to(self, priority, target, weight, mode):

		delta = target - priority
		priority_mod = priority + weight * delta

		if mode == +1:
			if priority_mod < priority:
				priority_mod = priority

		if mode == -1:
			if priority_mod > priority:
				priority_mod = priority

		return priority_mod

	def appetitive_response(self, gain):

		# this is called when an appetitive response is started
		# and responds by processing what was oriented into affect
		conf = self.system_state.priority_peak.source_conf
		value = np.sum(np.multiply(conf, self.pars.action.priority_peak_source_value))
		self.system_state.action_target_value = value * gain

	def get_inhibition(self):

		# return
		return self.interface.inhibition

	def set_priority(self, prio):

		# store
		self.interface.priority = prio

	def apply_push(self, push):

		# store
		self.parent.apply_push(push, self.retreatable)

	def apply_push_fovea(self, pushvec, flags=miro.constants.PUSH_FLAG_IMPULSE):

		# create push
		push = miro.utils.kc.KinematicPush()
		push.link = miro.constants.LINK_HEAD
		push.flags = flags
		push.pos = self.fovea_HEAD
		push.vec = pushvec

		# apply push
		self.apply_push(push)

		"""
	def get_push(self):

		# copy and clear applied push (may already be None)
		push = self.applied_push
		self.applied_push = None

		# return it
		return push
		"""

	def debug_event_start(self):

		if self.pars.flags.DEV_DEBUG_ACTION_PARAMS:
			print "(DEV_DEBUG_ACTION_PARAMS, at action start...)"
			print "\tpriority_peak.location", self.input.priority_peak.azim, self.input.priority_peak.elev
			print "\tpriority_peak.height", self.input.priority_peak.height
			print "\tpriority_peak.size", self.input.priority_peak.size
			print "\tpriority_peak.size_norm", self.input.priority_peak.size_norm
			print "\tpriority_peak.range", self.input.priority_peak.range

	def event_start(self):

		self.debug_event_start()

	def event_stop(self):

		pass

	def stop(self):

		# to stop action, stop motor plan clock (no effect if already stopped)
		self.clock.stop()

	def ascending(self):

		# default ascending() behaviour is to compute action priority

		# zero priority
		self.interface.priority = 0.0

		# if selected
		if self.interface.inhibition == 0:

			# raise priority if motor pattern is ongoing
			if self.clock.isActive():

				# ongoing priority
				self.interface.priority = self.ongoing_priority

		else:

			# compute priority from inputs
			self.interface.priority = self.compute_priority()

	def descending(self):

		# default descending() behaviour is to start/stop as required
		# and service an ongoing motor pattern if there is one

		# if motor plan is active
		if self.clock.isActive():

			# if inhibited
			if self.interface.inhibition > 0:

				# stop motor plan
				self.stop()

			# otherwise
			else:

				# service motor plan
				self.service()

		# if motor plan is not active
		else:

			# if disinhibited
			if self.interface.inhibition == 0:

				# debug
				if self.debug:
					self.input.dump()

				# start motor plan
				self.start()



class ActionInput(object):

	def __init__(self):

		# sensors
		self.user_touch = 0.0
		self.sonar_range = 0.0
		self.cliff = [0.0, 0.0]

		# priority peak
		self.priority_peak = signals.PriorityPeak()

		# mood and emotion
		self.emotion = signals.AffectState()
		self.sleep = signals.SleepState()

		# confidence in moving forward
		self.conf_surf = 1.0
		self.conf_space = 1.0

		# fixation
		self.fixation = 0.0

		# sensors
		self.wheel_speed_cmd = [0, 0]
		self.wheel_speed_opto = [0, 0]
		self.wheel_speed_back_emf = [0, 0]
		self.wheel_effort_pwm = [0, 0]

		# gaze
		self.dgaze = 0.0
		self.gaze_elevation = 0.0



class ActionInterface(object):

	def __init__(self):

		self.inhibition = 1.0
		self.priority = 0.0



class ActionClock(object):

	def __init__(self):

		self.steps_so_far = 0
		self.steps_total = 0
		self.T_norm = 0.0
		self.t_norm = 0.0

	def start(self, steps_total):

		# start integral clock
		self.steps_total = steps_total
		self.steps_so_far = 0

		# start normalised clock
		self.T_norm = 1.0 / steps_total
		self.t_norm = self.T_norm * 0.5

	def toString(self):

		return str(self.steps_so_far) + "/" + str(self.steps_total) + " (" + str(self.t_norm) + ")"

	def advance(self, auto_stop=False):

		# advance
		self.steps_so_far += 1
		self.t_norm += self.T_norm

		# auto stop at complete
		#
		# NB: both integral and norm clocks can overrun the expected time of
		# pattern safely; some patterns wait for another termination condition
		if auto_stop:
			if self.steps_so_far >= self.steps_total:
				self.stop()

	def stop(self):

		# reset clock
		self.steps_total = 0
		self.steps_so_far = 0
		self.T_norm = 0
		self.t_norm = 0

	def isActive(self):

		return self.steps_total > 0

	def cosine_profile(self):

		# return a profile from 0 to 1 (for t > steps_total, return 1)
		if self.t_norm >= 1.0:
			return 1.0

		return 0.5 - 0.5 * np.cos(self.t_norm * np.pi)

	def linear_profile(self):

		# Returns a profile from 0 to 1, for t > steps_total, return 1
		if self.t_norm >= 1.0:
			return 1.0

		return self.t_norm




