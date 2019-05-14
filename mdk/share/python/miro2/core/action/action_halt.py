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
#	Action Halt
#	Action to suddenly stop MIRO in the face of a sudden looming stimulus

import numpy as np
import miro2 as miro

from action_types import ActionTemplate



def fmt_wheels(x):

	s = "{0:.3f} {1:.3f}".format(x[0], x[1])
	return s

def constrain(x, a, b):

	for i in range(2):
		if x[i] < a:
			x[i] = a
		if x[i] > b:
			x[i] = b
	return x

def iir_filter(b, y, x):

	a = 1.0 - b
	y = a * y + b * x
	return y



class ActionHalt(ActionTemplate):

	def finalize(self):

		# parameters
		self.name = "halt"
		self.ongoing_priority = self.pars.action.priority_high
		self.modulate_by_wakefulness = False # halt must be allowed even when sleepy

		# reset
		self.reset()

	def reset(self):

		# initialise filters
		self.cmdf = np.array([0.0, 0.0])
		self.spdf = np.array([0.0, 0.0])
		self.efff = np.array([0.0, 0.0])
		self.accf = np.array([0.0, 0.0])
		self.stallf = np.array([0.0])
		self.cmdfa_bak = np.array([0.0, 0.0])

	def compute_priority(self):

		# extract
		cmd = np.array(self.input.wheel_speed_cmd)
		opt = np.array(self.input.wheel_speed_opto)
		emf = np.array(self.input.wheel_speed_back_emf)
		eff = np.array(self.input.wheel_effort_pwm)

		# report and exit, used for dev/wheel_stall
		if self.pars.flags.DEV_DEBUG_HALT:
			print \
				fmt_wheels(cmd), \
				fmt_wheels(opt), \
				fmt_wheels(emf), \
				fmt_wheels(eff)
			return 0.0

		# constrain signals at physically realisable
		eff = constrain(eff, -1, 1);
		cmd = constrain(cmd, -0.4, 0.4);

		# select speed signal
		spd = opt

		# input filters
		b = self.pars.action.halt_stall_input_filt
		self.cmdf = iir_filter(b, self.cmdf, cmd)
		self.spdf = iir_filter(b, self.spdf, spd)
		self.efff = iir_filter(b, self.efff, eff)

		# recover acceleration
		cmdfa = np.abs(self.cmdf)
		acc = cmdfa - self.cmdfa_bak
		self.cmdfa_bak = cmdfa
		acc = constrain(acc, 0.0, 1000000.0)

		# process
		spdp = np.abs(self.spdf);
		effp = np.abs(self.efff);

		# acceleration filter
		b = self.pars.action.halt_stall_acc_filt
		self.accf = iir_filter(b, self.accf, acc)

		# fixed gains reflect expected loads
		effp = effp * self.pars.action.halt_stall_eff_gain
		accf = self.accf * self.pars.action.halt_stall_acc_gain

		# additive detector
		stalls = effp - spdp - accf
		stall = np.mean(stalls)
		b = self.pars.action.halt_stall_output_filt
		self.stallf = iir_filter(b, self.stallf, stall)
		stalled = self.stallf > self.pars.action.halt_stall_thresh

		# stall?
		if self.pars.flags.ACTION_HALT_ON_STALL and stalled:
			return self.pars.action.priority_high

		# no stall
		return 0.0

	def start(self):

		# start pattern
		self.clock.start(self.pars.action.halt_num_steps)

	def event_start(self):

		# set halting state
		self.system_state.halting = True

		# reset
		self.reset()

	def event_stop(self):

		# set halting state
		self.system_state.halting = False

	def service(self):

		# read clock
		x = self.clock.cosine_profile()
		self.clock.advance(True)



