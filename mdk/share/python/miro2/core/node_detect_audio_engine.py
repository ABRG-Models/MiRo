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
import miro2 as miro

SAMP_PER_BLOCK=500
SAMP_BUFFERED=SAMP_PER_BLOCK*2



class DetectAudioEvent():

	def __init__(self, data):

		self.azim = data[0]
		self.elev = data[1]
		self.level = data[2]



class DetectAudioEngine():

	def __init__(self, pars, kc_s):

		self.pars = pars
		self.kc_s = kc_s

		# state
		self.n = SAMP_PER_BLOCK
		self.buf = None
		self.buf_abs = None
		self.buf_abs_fast = np.zeros((2, SAMP_PER_BLOCK), 'float32')
		self.buf_abs_slow = np.zeros((2, SAMP_PER_BLOCK), 'float32')
		self.buf_diff = np.zeros((SAMP_PER_BLOCK), 'float32')

		# best high point
		self.hn = 0

		# queue of not-yet-processed high points
		self.queue = []

		# output
		self.azim = 0.0
		self.elev = 0.0
		self.level = 0.0

	def filter(self, x, n):

		# create filter
		H = np.ones(n) / n

		# determine lead-in start point
		s = SAMP_PER_BLOCK - n

		# do filter
		y = np.vstack((
			np.convolve(H, x[0][s:], mode='full'),
			np.convolve(H, x[1][s:], mode='full')
		))

		# remove lead-in samples which are bunkum because
		# the filter was starting up, and lead-out samples
		# which we do not need
		y = y[:, n:-n+1]

		# ok
		return y

	def high_point(self, hn):

		# FOR INFO, see dev/jeffress

		# measurement range must be large enough to spot correlations at
		# time lags of interest. the ears are separated by about 6 samples
		# at 20kHz (104mm / 343m * 20k) but longer samples are needed to
		# do a good job at spotting inter-ear correlations.
		L_max = 6
		L = L_max * 8
		c = L # centre of xcorr equal to L in python 0-indexing (L+1 in matlab)

		# if out of range on left, discard, because this should only ever
		# happen during start up
		if (hn - L) < 0:
			return

		# if out of range on right, queue for next time
		if (hn + L) >= SAMP_BUFFERED:
			self.queue.append(hn)
			return

		# get height
		h = self.buf_diff[hn]

		# report
		#print hn-L, hn+L, h

		# extract section of original signal
		wav = self.buf[:, hn-L:hn+L+1]

		# xcorr
		xco = np.correlate(wav[0, :], wav[1, :], mode='same')

		# find best peak in xco (only in plausible range)
		i_peak = np.argmax(xco[c-L_max:c+L_max+1])
		i_peak += L - L_max

		# store
		"""
		np.savetxt('/tmp/wav', wav)
		x = self.buf_abs_slow[:, hn-L:hn+L+1]
		np.savetxt('/tmp/slow', x)
		x = self.buf_abs_fast[:, hn-L:hn+L+1]
		np.savetxt('/tmp/fast', x)
		x = self.buf_diff[hn-L:hn+L+1]
		np.savetxt('/tmp/dif', x)
		np.savetxt('/tmp/xco', xco)
		"""

		# discard if not actually a peak
		if xco[i_peak-1] > xco[i_peak] or xco[i_peak+1] > xco[i_peak]:
			#print "discard (not a peak)", xco[i_peak-2:i_peak+3]
			return

		# determine lag and level
		lag = float(i_peak - c)
		level = xco[i_peak]

		# discard if level is too low
		if level <= 0:
			print "discard (level too low)", level
			return

		# report
		#print "best peak", lag, level

		# adjust to sub-sample peak by fitting a parabola to three points
		y1 = xco[i_peak-1]
		y2 = xco[i_peak]
		y3 = xco[i_peak+1]
		dy1 = y2 - y1
		dy3 = y2 - y3
		den = dy1 + dy3
		#print y1, y2, y3, dy1, dy3

		# can only adjust if points are not nearly co-linear (if they are,
		# there would be no benefit anyway)
		if den > 1e-3:
			adj = 0.5 * (dy1 - dy3) / den
			lag = lag + adj

		# report
		#print "adjusted lag", lag

		# normalize and constrain
		lag *= (1.0 / self.pars.detect_audio.inter_ear_lag)
		lag = np.clip(lag, -1.0, 1.0)

		# report
		#print "normalized lag", lag

		# compute azimuth and RMS level
		azim = -np.arcsin(lag);
		level = np.sqrt(level / (2 * L + 1))

		# report
		#print "azimuth and level", azim, level

		# store
		if level > self.level:
			self.azim = azim
			self.level = level

	def process_configuration(self):

		# if running under perftest, this won't be available
		if self.kc_s is None:
			return

		# get locations of ears in HEAD
		loc_ear_l_HEAD = miro.utils.get("LOC_EAR_L_HEAD")
		loc_ear_r_HEAD = miro.utils.get("LOC_EAR_R_HEAD")

		# transform into FOOT
		loc_ear_l_FOOT = self.kc_s.changeFrameAbs(self.pars.LINK_HEAD, self.pars.LINK_FOOT, loc_ear_l_HEAD)
		loc_ear_r_FOOT = self.kc_s.changeFrameAbs(self.pars.LINK_HEAD, self.pars.LINK_FOOT, loc_ear_r_HEAD)

		# get point between ears at assumed height of noise sources
		x = 0.5 * (loc_ear_l_FOOT[0] + loc_ear_r_FOOT[0])
		y = 0.5 * (loc_ear_l_FOOT[1] + loc_ear_r_FOOT[1])

		# get azimuth of "dead-ahead" from ear locations
		dx = loc_ear_r_FOOT[0] - loc_ear_l_FOOT[0]
		dy = loc_ear_r_FOOT[1] - loc_ear_l_FOOT[1]
		azim = np.arctan2(dy, dx) # azimuth from ear_l to ear_r
		azim += np.pi * 0.5 # azimuth of dead-ahead

		# add azimuth of sound source from Jeffress computation
		azim += self.azim

		# estimate sound source location
		dx = np.cos(azim) * self.pars.detect_audio.assumed_sound_source_range
		dy = np.sin(azim) * self.pars.detect_audio.assumed_sound_source_range
		x_src = x + dx
		y_src = y + dy
		z_src = self.pars.detect_audio.assumed_sound_source_height

		# map that back into HEAD
		loc_src_FOOT = np.array([x_src, y_src, z_src])
		loc_src_HEAD = self.kc_s.changeFrameAbs(self.pars.LINK_FOOT, self.pars.LINK_HEAD, loc_src_FOOT)

		# recover from that the view line that we send as output
		#
		# (NB: this discards range and height information, which is
		# the right thing to do - they were useful only in estimation
		# of the view line, they are not useful to carry forward)
		x = loc_src_HEAD[0]
		y = loc_src_HEAD[1]
		z = loc_src_HEAD[2]
		r = np.sqrt(x*x + y*y)
		self.azim = np.arctan2(y, x)
		self.elev = np.arctan2(z, r)

	def process_data(self, data):

		# default
		event = None
		sound_level = []

		# clear any pending event (so we can send only one per block)
		self.level = 0.0

		# reshape
		data = np.asarray(data, 'float32') * (1.0 / 32768.0)
		data = data.reshape((4, SAMP_PER_BLOCK))

		# compute level
		sound_level = []
		for i in range(4):
			x = np.mean(np.abs(data[i]))
			sound_level.append(x)

		# beyond sound level, only interested in left & right
		data = data[0:2][:]

		# first fill buffer
		if self.buf is None:
			self.buf = data
			self.buf_abs = np.abs(data)
			return (event, sound_level)

		# roll buffers
		self.buf = np.hstack((self.buf[:, -SAMP_PER_BLOCK:], data))
		self.buf_abs = np.hstack((self.buf_abs[:, -SAMP_PER_BLOCK:], np.abs(data)))

		# since it is a rolling buffer, we can filter it in a rolling
		# manner. however, I don't know if the convolve() function
		# supports storing filter state. since we will use FIR filters,
		# we can get around this by filtering a little way back from
		# the roll point and using only the later part of the result.

		# filter slow for background level
		b = self.filter(self.buf_abs, 500)
		self.buf_abs_slow = np.hstack((self.buf_abs_slow[:, -SAMP_PER_BLOCK:], b))

		# filter fast for immediate level
		nf = 50
		i = self.filter(self.buf_abs, nf)
		self.buf_abs_fast = np.hstack((self.buf_abs_fast[:, -SAMP_PER_BLOCK:], i))

		# diff those two to see events
		d = np.mean(i - b, axis=0)
		self.buf_diff = np.hstack((self.buf_diff[-SAMP_PER_BLOCK:], d))

		# process any queued high points
		for hn in self.queue:
			self.high_point(hn - SAMP_PER_BLOCK)
		self.queue = []

		# continue reading through looking for events
		N = SAMP_PER_BLOCK * 2
		d = self.buf_diff
		n = self.n - SAMP_PER_BLOCK
		hn = self.hn
		if hn > -1:
			if hn >= SAMP_PER_BLOCK:
				hn -= SAMP_PER_BLOCK
			else:
				# high point now forgotten
				hn = 0
		thresh = self.pars.detect_audio.raw_magnitude_thresh
		if hn >= 0:
			h = d[hn]
		else:
			h = thresh

		# loop through samples
		while n < N:

			# if waiting for reset
			if hn == -1:

				# if well below threshold
				if d[n] < (0.5 * thresh):

					# store new high point (below threshold)
					h = d[n]
					hn = n

			# if not waiting for reset
			else:

				# look for high point
				if d[n] > h:

					# update stored high point
					h = d[n]
					hn = n

				# look for end of high point
				if h > thresh and d[n] < (0.5 * h):

					# process high point
					self.high_point(hn - nf / 2)

					# clear
					h = thresh
					hn = -1

			# advance
			n += 1

		# restore
		self.n = n
		self.hn = hn

		# default
		event = None

		# process any pending event
		if self.level:

			# adjust for configuration of robot right now
			self.process_configuration()

			# publish
			event = DetectAudioEvent([self.azim, self.elev, self.level])

		# return
		return (event, sound_level)



