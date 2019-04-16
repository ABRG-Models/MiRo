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



class PriorityPeak(object):

	def __init__(self, stream_index=0, height=0.0, size=0.0, azim=0.0, elev=0.0):

		# from source
		self.stream_index = stream_index
		self.height = height
		self.size = size # size normalized against frame size
		self.azim = azim
		self.elev = elev

		# calculated / inferred
		self.source_conf = np.array([0.0, 0.0])
		self.source_range = np.array([0.0, 0.0])
		self.size_norm = 0.0 # size normalized against parameter "size_large"
		self.volume = 0.0
		self.range = 0.0

	def finalize(self, pars):

		# get normalized size
		self.size_norm = np.clip(self.size * pars.action.size_large_recip, 0.0, 1.0)

		# compute volume at input
		self.volume = self.size_norm * self.height

		# estimate range based purely on peak size (very approximate, see dev/estimate_range)
		if self.size == 0:
			range_from_size = pars.action.range_estimate_max
		else:
			r = 0.05 / np.sqrt(self.size)
			range_from_size = np.clip(r, pars.action.range_estimate_min, pars.action.range_estimate_max)

		# estimate summary range across possible sources of information
		conf = copy.copy(self.source_conf)
		range = np.append(self.source_range, range_from_size)
		x = np.sum(conf)
		if x < 1.0:
			conf = np.append(conf, 1.0 - x)
		else:
			conf = np.append(conf / x, 0.0)
		self.range = np.sum(np.multiply(conf, range))

	def as_string(self):
		return "[stream_index=" + str(self.stream_index) \
			+ ",source=" + str(self.source) \
			+ ",height=" + str(self.height) \
			+ ",size=" + str(self.size) \
			+ ",azim=" + str(self.azim) \
			+ ",elev=" + str(self.elev) \
			+ "]"



class AffectState(object):

	def __init__(self, valence=0.5, arousal=0.5):
		self.valence = valence
		self.arousal = arousal

	def drive(self, driver):
		self.valence += driver.gamma * driver.target[0]
		self.arousal += driver.gamma * driver.target[1]



class SleepState(object):

	def __init__(self, pressure=0.0, wakefulness=1.0):
		self.wakefulness = wakefulness
		self.pressure = pressure

	def drive(self, driver):
		self.wakefulness += driver.gamma * driver.target[0]
		self.pressure += driver.gamma * driver.target[1]




