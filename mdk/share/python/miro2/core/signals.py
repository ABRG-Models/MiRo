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
import copy
import miro2 as miro



class PriorityPeak(object):

	def __init__(self, stream_index=0, loc_d=None, height=0.0, size=0.0, azim=0.0, elev=0.0):

		# from source
		self.stream_index = stream_index
		self.loc_d = loc_d # location in original frame
		self.height = height
		self.size = size # size normalized against frame size
		self.azim = azim
		self.elev = elev

		# calculated / inferred
		self.source_conf = []
		self.source_range = []
		self.source_value = []
		self.size_norm = 0.0 # size normalized against parameter "size_large"
		self.volume = 0.0
		self.range = 0.0
		self.value = 0.0

		# debug information
		self.actioned = -1

	def append_source(self, conf, range, value):

		self.source_conf.append(conf)
		self.source_range.append(range)
		self.source_value.append(value)

	def finalize(self, pars):

		# get normalized size
		self.size_norm = np.clip(self.size * pars.action.size_large_recip, 0.0, 1.0)

		# compute volume at input
		self.volume = self.size_norm * self.height

		# compute appetitive value
		self.value = np.sum(np.multiply(self.source_conf, self.source_value))

		# gather confidence across explicitly provided sources
		conf_sum = np.sum(self.source_conf)

		# if that's below unity, add the range_from_size as an additional source
		if conf_sum < 1.0:

			# estimate range based purely on peak size (very approximate, see dev/estimate_range)
			if self.size == 0:
				range_from_size = pars.action.range_estimate_max
			else:
				r = 0.05 / np.sqrt(self.size)
				range_from_size = np.clip(r, pars.action.range_estimate_min, pars.action.range_estimate_max)

			# append that (note "value" is irrelevant and not used)
			self.append_source(1.0 - conf_sum, range_from_size, 0.0)

		# normalise confidence
		self.source_conf = np.array(self.source_conf) * (1.0 / np.sum(self.source_conf))

		# estimate range across sources of information
		self.range = np.sum(np.multiply(self.source_conf, self.source_range))

		"""
		At this stage, the peak location (view line) is defined in a space that
		is stream-specific:

			stream 0: "vh" for caml, which has the same orientation as HEAD but
				the view line starts from the optical centre of caml.

			stream 1: as stream 0, for camr.

			stream 2: the view line is notionally defined in HEAD, but in fact
				since the source is currently always audio, it's in practice
				centred on a point between the two ear microphones, we just
				don't worry about that degree of accuracy.

		We want to transform from each of these spaces into HEAD - this is
		possible only if we have a decent range estimate.
		"""

		"""
		This would be great, but at the moment it doesn't work /that/ well - try
		it in motion_calib.world. I think possibly because the mapping from pixel
		space to azimuth is already not linear, and we're currently using a linear
		map there. So, for now, we'll not do this, which means orienting will be
		to a gaze line moving up and forward from one or other eye (whichever was
		the one that gathered the sensory stimulus in the first place). That's not
		bad - the user won't be able to tell, since the location differs only by
		4.3cm from the true location. Note that for audio stimuli, there is also
		a bias, but since audio stimuli are so poorly localised it doesn't matter,
		so we leave them be.

		# move view line into HEAD
		if self.stream_index < 2:

			# recover target position in "offset HEAD" using estimated range
			xyz = miro.utils.kc_interf.kc_viewline_to_position(self.azim, self.elev, self.range)

			# move from "offset HEAD" into HEAD by adding camera location
			xyz += pars.camera.location[self.stream_index]

			# recover viewline in HEAD
			(self.azim, self.elev) = miro.utils.kc_interf.kc_position_to_viewline(xyz)
		"""

	def as_string(self):
		return "[stream_index=" + str(self.stream_index) \
			+ ",source=" + str(self.source) \
			+ ",height=" + str(self.height) \
			+ ",size=" + str(self.size) \
			+ ",azim=" + str(self.azim) \
			+ ",elev=" + str(self.elev) \
			+ "]"



def adjustState(state, adjust):

	if adjust.gamma == -1.0:
		state += adjust.data
	elif adjust.gamma >= 0.0 and adjust.gamma <= 1.0:
		state += adjust.gamma * (adjust.data - state)

	return np.clip(state, 0.0, 1.0)



class AffectState(object):

	def __init__(self, valence=0.5, arousal=0.5):
		self.valence = valence
		self.arousal = arousal

	def adjust(self, adjust):
		self.valence = adjustState(self.valence, adjust.valence)
		self.arousal = adjustState(self.arousal, adjust.arousal)



class SleepState(object):

	def __init__(self, pressure=0.0, wakefulness=1.0):
		self.wakefulness = wakefulness
		self.pressure = pressure

	def adjust(self, adjust):
		self.wakefulness = adjustState(self.wakefulness, adjust.wakefulness)
		self.pressure = adjustState(self.pressure, adjust.pressure)




