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
#	Action Special
#	special actions in response to coded stimuli (MIROcube faces)

import os
import rospy
import copy
import numpy as np
import random

import miro2 as miro

from action_types import ActionTemplate

# messages larger than this will be dropped by the receiver
MAX_STREAM_MSG_SIZE = (4096 - 48)

# amount to keep the buffer stuffed - larger numbers mean
# less prone to dropout, but higher latency when we stop
# streaming. with a read-out rate of 8k, 2000 samples will
# buffer for quarter of a second, for instance.
BUFFER_STUFF_BYTES = 4000

# patterns
ID_SPIN = 1
ID_AVERT = 2
ID_CIRCLE = 3
ID_TAICHI = 4
ID_ADA = 5
ID_WORSHIP = 6



class ActionSpecial(ActionTemplate):

	def finalize(self):

		# parameters
		self.name = "special"
		self.tag_id = 0
		self.ongoing_priority = self.pars.action.priority_uninterruptable # since the actions are all FAPs, they must complete
		self.audio_data = None

		# initialize audio
		self.audio_init()

	def estimate_range(self, size_in_pix, size_in_m):

		# first, convert size_in_pix to normalised image size
		size_norm = float(size_in_pix) / self.pars.decode.image_width

		# normalise that by known size of object
		size_rel = size_norm / size_in_m

		# we could then retrieve the range from theory, but rather
		# than bother to actually figure it out (it is somewhat
		# dependent on the camera distortion model) I'm just going
		# to estimate it empirically for now
		if size_rel > 0.0:
			range = 0.5 / size_rel
			#print "est range", size_in_pix, size_in_m, range
		else:
			range = self.pars.action.range_estimate_max

		if range < self.pars.action.range_estimate_min:
			range = self.pars.action.range_estimate_min
		if range > self.pars.action.range_estimate_max:
			range = self.pars.action.range_estimate_max

		# ok
		return range

	def compute_priority(self):

		# if flagged off
		if not self.pars.flags.ACTION_ENABLE_SPECIAL:
			return 0.0

		# strongest signal
		prio = 0.0
		self.tag_id = 0
		self.stream_index = 0
		self.vh = None

		# for each stream
		for stream_index in range(2):

			# if present
			if self.system_state.detect_objects_for_50Hz[stream_index] is None:
				continue

			# for each tag
			for tag in self.system_state.detect_objects_for_50Hz[stream_index].tags:

				# extract tag
				id = tag.id
				cen = [tag.centre[0], tag.centre[1]]
				corn = [tag.corners[0:2], tag.corners[2:4], tag.corners[4:6], tag.corners[6:8]]

				# move coordinates into frame pixels
				cam = self.system_state.camera_model_mini
				cen = cam.d2p(cen)
				c0 = cam.d2p(np.array(corn[0]))
				c1 = cam.d2p(np.array(corn[1]))
				c2 = cam.d2p(np.array(corn[2]))
				c3 = cam.d2p(np.array(corn[3]))

				# get azimuth/elevation in HEAD
				vh = cam.p2vh(cen, stream_index)
				oh = cam.vh2oh(stream_index, vh, 1.0)

				# get size
				d1 = np.linalg.norm(c2 - c0)
				d2 = np.linalg.norm(c3 - c1)
				d = np.max([d1, d2])
				d_min = np.min([d1, d2])

				# get tilt angle
				tilt = d_min / d

				# get parameters
				x = int(cen[0])
				y = int(cen[1])
				r = int(d/2.0)

				# get range
				range_to_obj = self.estimate_range(d * 0.707, self.pars.action.april_size_m)

				# compute magnitude
				gain = self.pars.spatial.april_gain_at_1m
				if range_to_obj < 0.1:
					range_to_obj = 0.1
				gain *= 1.0 / np.sqrt(range_to_obj)
				gain = np.clip(gain, 0.0, 1.0)
				mag = gain

				# modulate by tilt so more face-on tags take priority
				mag *= tilt

				# use magnitude as priority
				if mag > prio:
					prio = mag
					self.tag_id = id
					self.stream_index = stream_index
					self.vh = vh
					self.oh = oh

		return prio

	def start(self):

		# decide pattern rate / time
		"""
		elev = self.input.priority_peak.elev - self.input.gaze_elevation
		rad = np.sqrt(np.square(self.input.priority_peak.azim) + np.square(elev))
		sec_ideal = rad * self.pars.action.orient_speed_sec_per_rad
		steps_ideal = int(sec_ideal * self.pars.timing.tick_hz)
		steps = np.clip(steps_ideal, self.pars.action.orient_min_steps, self.pars.action.orient_max_steps)
		"""

		# default
		steps = 50

		# random polarity
		self.random_polarity = -1.0 + 2.0 * random.randint(0, 1)

		# stream index based polarity
		self.stream_polarity = +1.0 - 2.0 * self.stream_index

		# compute vf (viewline in FOOT)
		of = self.kc.changeFrameAbs(miro.constants.LINK_HEAD, miro.constants.LINK_FOOT, self.oh)
		az = np.arctan2(of[1], of[0])
		el = np.arctan2(of[2], np.sqrt(of[0]*of[0] + of[1]*of[1]))
		self.vf = miro.utils.camera_model.ViewLine(az, el)

		# azimuth based polarity
		self.azim_polarity = np.sign(self.vf.azim)

		# pattern: spin on the spot
		if self.tag_id == ID_SPIN:
			steps = 100

			# +ve valence boost
			self.system_state.action_target_valence = 0.5

		# pattern: draw circle with fovea
		if self.tag_id == ID_CIRCLE:
			steps = 200

			# +ve valence boost
			self.system_state.action_target_valence = 0.2

		# pattern: tai-chi
		if self.tag_id == ID_TAICHI:
			steps = 1000

			# +ve valence boost
			self.system_state.action_target_valence = 0.2

		# pattern: worship
		if self.tag_id == ID_WORSHIP:
			steps = 150

			# +ve valence boost
			self.system_state.action_target_valence = 0.5

			# also has a calming effect
			self.system_state.action_target_arousal = -0.25

		# pattern: turn away
		if self.tag_id == ID_AVERT:
			steps = 100

			# -ve valence boost
			self.system_state.action_target_valence = -0.5

		# pattern: ada
		if self.tag_id == ID_ADA:
			if self.audio_data:
				steps = self.audio_start()

		# start action clock
		self.clock.start(steps)

	def audio_init(self):

		# find src file
		file_src = miro.utils.get_media_file("ada_lovelace.mp3")
		if not file_src:
			return

		# decode mp3
		file_dst = "/tmp/action_special_audio_data.decode"
		if not os.path.isfile(file_dst):
			cmd = "ffmpeg -y -i " + file_src + " -f s16le -acodec pcm_s16le -ar 8000 -ac 1 " + file_dst
			os.system(cmd)
			if not os.path.isfile(file_dst):
				# fail silently
				self.audio_data = None
				print "failed decode mp3 in action_special"
				return

		# load wav
		with open(file_dst, 'rb') as f:
			dat = f.read()

		# convert to numpy array
		dat = np.fromstring(dat, dtype='int16').astype(np.int32)

		# normalise wav
		dat = dat.astype(np.float)
		sc = 32767.0 / np.max(np.abs(dat))
		dat *= sc

		# store ready for use
		self.audio_data = dat.astype(np.int16).tolist()

	def audio_start(self):

		# state
		self.audio_data_r = 0

		# return amount of time to wait after audio complete
		return 100

	def audio_play(self):

		# NB: because we keep the buffer more stuffed than node_voice, no
		# output comes from node_voice whilst we are running - effectively
		# we over-ride the voice output with this stream.

		# if we've heard from streamer
		if self.input.stream:

			# get streamer state
			buffer_space = self.input.stream[0]
			buffer_total = self.input.stream[1]

			# clear that so we won't over-stuff if
			# the streamer state doesn't reach us
			# in a timely manner
			self.input.stream = None

			# compute amount to send
			buffer_rem = buffer_total - buffer_space
			n_bytes = BUFFER_STUFF_BYTES - buffer_rem
			n_bytes = max(n_bytes, 0)
			n_bytes = min(n_bytes, MAX_STREAM_MSG_SIZE)

			# if amount to send is non-zero, send it
			if n_bytes > 0:

				self.system_output.stream = self.audio_data[self.audio_data_r:self.audio_data_r+n_bytes]
				self.audio_data_r += n_bytes

		# regardless, if some left to send, reset clock
		if self.audio_data_r < len(self.audio_data):
			self.clock.reset()

	def service(self):

		# get constants
		c = miro.constants

		# read clock
		x = self.clock.cosine_circle_profile()
		y = self.clock.sine_profile()
		z = self.clock.cosine_profile()
		self.clock.advance(True)

		# pattern: spin on the spot
		if self.tag_id == ID_SPIN:

			# push body
			q = x * 0.1 * self.azim_polarity
			self.apply_push_body(np.array([0.0, q, 0.0]))

			# look straight out
			q = np.clip(x * 3.0, 0.0, 1.0)
			config = self.kc.getConfig()
			config[1] = c.LIFT_RAD_MIN + q * (c.LIFT_RAD_MAX - c.LIFT_RAD_MIN)
			config[2] = 0.0
			config[3] = c.PITCH_RAD_MAX + q * (c.PITCH_RAD_MIN - c.PITCH_RAD_MAX)
			self.kc.setConfig(config)

		# pattern: circle
		if self.tag_id == ID_CIRCLE:

			# circle
			config = self.kc.getConfig()
			config[1] = c.LIFT_RAD_MIN + x * (c.LIFT_RAD_MAX - c.LIFT_RAD_MIN)
			config[2] = c.YAW_RAD_MAX * y
			config[3] = c.PITCH_RAD_MAX + x * (c.PITCH_RAD_MIN - c.PITCH_RAD_MAX)
			print config
			self.kc.setConfig(config)

			# open eyes
			self.system_output.cosmetic_joints[2] = 0.0
			self.system_output.cosmetic_joints[3] = 0.0

		# pattern: tai-chi
		if self.tag_id == ID_TAICHI:

			# quadrant time
			t = self.clock.t_norm * 4.0
			q = int(t)
			t -= q

			# profiles
			x = 0.5 - 0.5 * np.cos(t * np.pi)

			# do quadrant
			config = self.kc.getConfig()
			if q == 0:
				config[1] = c.LIFT_RAD_MIN + x * (c.LIFT_RAD_MAX - c.LIFT_RAD_MIN)
				config[2] = 0.0
				config[3] = c.PITCH_RAD_MIN + x * (c.PITCH_RAD_MAX - c.PITCH_RAD_MIN)
			if q == 1:
				config[2] = c.YAW_RAD_MAX * x * self.random_polarity
			if q == 2:
				config[1] = c.LIFT_RAD_MIN + (1.0 - x) * (c.LIFT_RAD_MAX - c.LIFT_RAD_MIN)
				config[3] = c.PITCH_RAD_MIN + (1.0 - x) * (c.PITCH_RAD_MAX - c.PITCH_RAD_MIN)
			if q == 3:
				config[2] = c.YAW_RAD_MAX * (1.0 - x) * self.random_polarity
			self.kc.setConfig(config)

			# close eyes
			self.system_output.cosmetic_joints[2] = 1.0
			self.system_output.cosmetic_joints[3] = 1.0

		# pattern: worship
		if self.tag_id == ID_WORSHIP:

			# worship
			config = self.kc.getConfig()
			config[1] = c.LIFT_RAD_MIN + x * (c.LIFT_RAD_MAX - c.LIFT_RAD_MIN)
			config[2] = self.vf.azim # set from azimuth of tag detection in FOOT
			config[3] = c.PITCH_RAD_MAX + x * (c.PITCH_RAD_MIN - c.PITCH_RAD_MAX)
			self.kc.setConfig(config)

			# close eyes
			self.system_output.cosmetic_joints[2] = x
			self.system_output.cosmetic_joints[3] = x

		# pattern: turn away
		if self.tag_id == ID_AVERT:

			# push body
			q = x * 0.1 * -self.azim_polarity
			if self.clock.steps_so_far > 50:
				q = 0.0
			self.apply_push_body(np.array([0.0, q, 0.0]))

			# look straight out
			q = np.clip(x * 3.0, 0.0, 1.0)
			config = self.kc.getConfig()
			config[1] = c.LIFT_RAD_MIN + q * (c.LIFT_RAD_MAX - c.LIFT_RAD_MIN)
			config[2] = 0.0
			config[3] = c.PITCH_RAD_MAX + q * (c.PITCH_RAD_MIN - c.PITCH_RAD_MAX)
			self.kc.setConfig(config)

			# close eyes
			self.system_output.cosmetic_joints[2] = 1.0
			self.system_output.cosmetic_joints[3] = 1.0

		# pattern: ada
		if self.tag_id == ID_ADA:

			if self.audio_data:

				# profile
				q = self.clock.linear_profile()

				# hold upright position as if channeling
				config = self.kc.getConfig()
				config[1] = c.LIFT_RAD_MIN # + (q * 0.5) * (c.LIFT_RAD_MAX - c.LIFT_RAD_MIN)
				config[2] = 0.0
				config[3] = c.PITCH_RAD_MAX # + (q * 0.5) * (c.PITCH_RAD_MAX - c.PITCH_RAD_MIN)
				self.kc.setConfig(config)

				# eyes half open
				e = (1.0 - q) * 0.75
				self.system_output.cosmetic_joints[2] = e
				self.system_output.cosmetic_joints[3] = e

				# play
				self.audio_play()
