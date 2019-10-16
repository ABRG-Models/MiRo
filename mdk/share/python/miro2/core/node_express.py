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

import numpy as np
import time

import node

import miro2 as miro



class ExpressLight(object):

	def __init__(self, red=0, grn=0, blu=0, dphase=0):
		self.red = red
		self.grn = grn
		self.blu = blu
		self.dphase = dphase



class NodeExpress(node.Node):

	def __init__(self, sys):

		node.Node.__init__(self, sys, "express")

		# init state machines
		self.ears_init()
		self.eyelids_init()
		self.tail_init()
		self.lights_init()

	def lights_init(self):

		self.light = ExpressLight()

		self.breathing_phase = 0
		self.breathing_phase_local = 0

	def lights_tick(self):

		# if enabled
		if self.pars.flags.EXPRESS_THROUGH_LIGHT:

			# local breathing phase used for dev
			self.breathing_phase_local += 1
			if self.breathing_phase_local >= 3:
				self.breathing_phase += 0.1
				if self.breathing_phase >= (np.pi * 2.0):
					self.breathing_phase = 0.0

			# colour
			valence = self.valence * 2.0 - 1.0
			self.light.red = min(255, 255 - valence * 255)
			self.light.grn = min(255, 255 + valence * 255)
			self.light.blu = max(0, 255 - abs(valence) * 2 * 255)

			# normalise luminance
			lum = int(self.light.red + self.light.grn + self.light.blu)
			self.light.red = int(self.light.red * 255 / lum)
			self.light.grn = int(self.light.grn * 255 / lum)
			self.light.blu = int(self.light.blu * 255 / lum)

			# if sleepy, approach orange
			wa = self.wakefulness
			wo = 1.0 - wa
			self.light.red = int(self.pars.express.ORANGE_red * wo + self.light.red * wa)
			self.light.grn = int(self.pars.express.ORANGE_grn * wo + self.light.grn * wa)
			self.light.blu = int(self.pars.express.ORANGE_blu * wo + self.light.blu * wa)

			# constrain color channels
			self.light.red = np.clip(self.light.red, 0, 255)
			self.light.grn = np.clip(self.light.grn, 0, 255)
			self.light.blu = np.clip(self.light.blu, 0, 255)

			# compute phase of each lamp
			inhale_phase = np.cos(self.breathing_phase) * self.pars.express.led_phase_range
			phase = [
					inhale_phase - self.pars.express.led_phase_separation,
					inhale_phase,
					inhale_phase + self.pars.express.led_phase_separation
					]

			# for each of three lamps
			for i in range(0, 3):

				# compute amplitude of this lamp
				p = phase[i]
				amp = np.uint32(np.clip(np.cos(p), 0.0, 1.0) * 255.0)

				# construct ARGB value
				val = amp << 24 | np.uint32(self.light.red) << 16 | \
						np.uint32(self.light.grn) << 8 | np.uint32(self.light.blu)

				# lay in
				self.output.illum[i] = val
				self.output.illum[i+3] = val

		# if not enabled
		else:

			# for each of three lamps
			for i in range(0, 3):

				# lay in
				self.output.illum[i] = 0
				self.output.illum[i+3] = 0

	def eyelids_init(self):

		self.time_to_next_blink = 0
		self.blink_time = 0
		self.blink_period = 0

	def eyelids_tick(self):

		# if enabled
		if self.pars.flags.EXPRESS_THROUGH_EYELIDS:

			# default eyelid closure is based on wakefulness
			eyelid_opening = self.wakefulness

			# if user is touching MiRo, and he's in a good mood, eyes droop appreciatively
			droop = np.tanh(self.user_touch * 3.0) * (self.valence ** 3.0)
			eyelid_opening -= droop * self.pars.express.eyelids_droop_on_touch * eyelid_opening

			# if halting, that's the highest priority
			if self.state.halting:
				eyelid_opening = 0.0

			# else we'll consider blinking
			else:

				# if fairly awake (eyes not too drooped already)
				if self.wakefulness > 0.5:

					# if timer is at zero, compute time to next blink
					if self.time_to_next_blink == 0:

						# if not in motion, save blink for when we move
						# so as not to upset motion detection unnecessarily
						if not self.state.in_motion:

							# do nothing
							pass

						else:

							# if blink is not active yet
							if self.blink_period == 0:

								# decide to blink or double blink
								if np.random.uniform() < self.pars.express.double_blink_prob:
									self.blink_period = self.pars.express.double_blink_period
									self.blink_mult = float(2)
									self.blink_time = 0
								else:
									self.blink_period = self.pars.express.blink_period
									self.blink_mult = float(1)
									self.blink_time = 0

							else:

								# implement blink
								t = self.blink_mult * self.blink_time / self.blink_period
								t = np.mod(t, 1)
								self.blink_time += 1

								# set eyelid position
								if t < 0.5:
									eyelid_opening = 0

								# if blink is finished
								if self.blink_time == self.blink_period:

									# clear blink
									self.blink_period = 0

									# compute new time_to_next_blink
									p = np.random.gamma(3, 0.333)
									q = self.pars.express.blink_mean_interval
									r = self.pars.express.blink_refractory_period
									self.time_to_next_blink = int(p * (q - r) + r)

					# else
					else:

						# wait for next blink time
						self.time_to_next_blink -= 1

		# if not enabled
		else:

			# direct
			eyelid_opening = 1.0

		# lay in
		self.output.cosmetic_joints[2] = 1.0 - eyelid_opening
		self.output.cosmetic_joints[3] = 1.0 - eyelid_opening

	def tail_init(self):

		self.wagging = False
		self.tail_wag_pos = 0.5
		self.wag_dir = 1

	def tail_tick(self):

		# if enabled
		if self.pars.flags.EXPRESS_THROUGH_TAIL:

			valence = self.valence * 2.0 - 1.0
			arousal = self.arousal * 2.0 - 1.0

			droop = np.clip(-valence, 0.0, 1.0)
			wag = 0.5 * (arousal + valence)

			# Wag tail more slowly when tired? Possibly
			wag *= self.wakefulness

			# if recent touch
			if wag > 0 and self.user_touch:

				# if currently wagging
				if self.wagging:

					# stop wagging
					if wag <= 0:
						self.tail_wag_pos = 0.5
						self.wagging = False

					# continue wagging
					else:
						x = wag * self.pars.express.tail_wag_max_amp * 0.5
						top_lim = 0.5 + x
						bot_lim = 0.5 - x
						if self.tail_wag_pos > top_lim:
							self.wag_dir = - 1
						elif self.tail_wag_pos < bot_lim:
							self.wag_dir = 1

						self.tail_wag_pos += self.wag_dir * wag / 20.0

				else:

					# start wagging
					if wag > 0:
						self.wagging = True
					else:
						self.tail_wag_pos = 0.5

			else:

				self.tail_wag_pos = 0.5

			# lay in
			self.output.cosmetic_joints[0] = droop
			self.output.cosmetic_joints[1] = self.tail_wag_pos

		# if not enabled
		else:

			# lay in
			self.output.cosmetic_joints[0] = 0.0
			self.output.cosmetic_joints[1] = 0.5

	def ears_init(self):

		# nowt to do
		pass

	def ears_tick(self):

		# if enabled
		if self.pars.flags.EXPRESS_THROUGH_EARS:

			# map valence to ears
			earpos = (1.0 - self.valence)

		# if not enabled
		else:

			# direct
			earpos = 0.5

		# lay in
		self.output.cosmetic_joints[4] = earpos
		self.output.cosmetic_joints[5] = earpos

	def tick(self):

		voice_state_avail = not self.input.voice_state is None

		if voice_state_avail:
			self.breathing_phase_local = 0
			self.breathing_phase = self.input.voice_state.breathing_phase

			# used up
			self.input.voice_state = None

		#DebugVoiceState
		"""
		t = time.time() - 1565096267;
		x = "2 " + str(voice_state_avail) + " " + str(t) + " " + str(self.breathing_phase) + "\n"
		with open("/tmp/voice_state", "a") as file:
			file.write(x)
		"""

		# init inputs
		self.user_touch = self.state.user_touch
		self.valence = self.state.emotion.valence
		self.arousal = self.state.emotion.arousal
		self.wakefulness = self.state.wakefulness

		# tick
		self.lights_tick()
		self.tail_tick()
		self.ears_tick()
		self.eyelids_tick()




