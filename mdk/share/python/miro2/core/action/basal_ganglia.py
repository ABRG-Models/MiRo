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
#	Basal ganglia model for action selection


import rospy
import numpy as np
import miro2 as miro

# Set threshold required for a valid MULL selection
# 0.1001 seems to be a good compromise that allows MULL to win in the sim a reasonable amount of the time
MULL_VAL = 0.1001


class BasalGanglia(object):

	def __init__(self, pars):

		# store
		self.pars = pars.selection

		# start with "no action" selected
		self.selected = -1

	def update(self, actions):

		prios = np.zeros(len( actions))

		for i, action in enumerate(actions):

			prio = action.interface.priority

			# Hysteretical feedback term
			# if i == self.selected:
			# Exclude MULL from hysteresis
			if (i == self.selected) and (i != 0):
				prio += self.pars.selection_hysteresis

			prio = np.clip(prio, 0.0, 1.0)
			prios[i] = prio


		# normalise output
		#if np.sum(prios) > 0:
		#	prios /= np.sum(prios)

		# Add a little noise
		noise = np.random.normal(size=len(actions)) * self.pars.selection_noise_mag
		prios += abs(noise)

		# If MULL is the winning action at a threshold lower than MULL_VAL,
		# add noise to non-MULL actions and subtract noise from MULL until some action is greater than 0.1
		while (np.argmax(prios) == 0) and (prios[0] < MULL_VAL) and (all(i < 0.1 for i in prios[1:])):
			noise = np.random.normal(size=len(actions) - 1) * self.pars.selection_noise_mag
			prios[1:] += abs(noise)
			prios[0] -= abs(np.random.normal() * self.pars.selection_noise_mag)

		# Select action with maximum priority (winner take all)
		selected = np.argmax(prios)

		# update selection
		if self.selected != selected:
			print "\n[**** SELECT ACTION", actions[selected].name, "****]"
			actions[self.selected].event_stop()
			self.selected = selected
			actions[self.selected].event_start()

		# feed back inhibition to each action sub-system
		for i, action in enumerate(actions):

			if i == self.selected:
				action.interface.inhibition = 0.0
			else:
				action.interface.inhibition = 1.0