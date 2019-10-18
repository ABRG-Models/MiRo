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
#	Basal ganglia model for action selection


import rospy
import numpy as np
import random

import miro2 as miro



def fmt_prio(prio):

	np.set_printoptions(precision=3, suppress=True)
	s = prio.__str__()
	np.set_printoptions(precision=8, suppress=False)
	return s

class BasalGanglia(object):

	def __init__(self, state, pars):

		# store
		self.state = state
		self.pars = pars

		# start with "no action" selected
		self.prio = []
		self.inhib = []
		self.selected = -1

		# and not in sham mode
		self.sham = False

	def update(self, node):

		actions = node.actions

		self.prio = np.zeros(len(actions))
		self.inhib = np.ones(len(actions))

		for i, action in enumerate(actions):

			prio = action.interface.priority

			# Hysteretical feedback term
			if	i == self.selected:
				prio += self.pars.selection.selection_hysteresis

			prio = np.clip(prio, 0.0, 1.0)
			self.prio[i] = prio

		#print self.prio

		# normalise output
		#if np.sum(self.prio) > 0:
		#	self.prio /= np.sum(self.prio)

		# Add a little noise
		noise = np.random.normal(size=len(actions)) * self.pars.selection.selection_noise_mag
		self.prio += noise

		# select action with maximum priority (winner take all)
		selected = np.argmax(self.prio)

		# update selection
		if self.selected != selected:

			# report
			print "[**** SELECT ACTION", actions[selected].name, "@", self.state.tick, fmt_prio(self.prio), "****]"
			actions[self.selected].event_stop()
			self.selected = selected
			actions[self.selected].event_start()

			# do cliff sensor fail detect
			node.cliff_sensor_fail_detect(actions[self.selected])

			# recompute sham state
			if self.pars.action.action_prob == 1.0:
				# do not roll dice or annoy dev with message
				self.sham = False
			else:
				u = random.uniform(0.0, 1.0)
				self.sham = u > self.pars.action.action_prob
				print "\tsham state is now", self.sham

		# feed back inhibition to each action sub-system
		for i, action in enumerate(actions):

			if i == self.selected:
				action.interface.inhibition = 0.0
				self.inhib[i] = 0.0
			else:
				action.interface.inhibition = 1.0
