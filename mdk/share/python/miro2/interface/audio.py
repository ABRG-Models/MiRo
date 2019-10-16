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
from collections import deque
import math

class ClapDetector:

	def __init__(self):

		# parameters
		self.background_lambda = 0.1 # IIR filter
		self.sample_time = 0.025	#Blocks recieved at 40hz
		self.max_clap_samples = 0.15/self.sample_time #Max number of samples before considered too long for clap
		self.t0 = 0.0
		
		# state
		self.clap_mean = 0.0
		self.background_mean = 0.02 # initialise conservatively
		self.over_threshold_count = 0 #Count for number of samples which exceeded volume threshold
		self.blocks_since_clap = 1 #Count number of samples since last clap
		self.time_at_clap = -60.0 # some large negative value
		self.clap_seen = False
		
	def have_seen_clap(self):
		if self.clap_seen:
			self.clap_seen = False
			return True
		return False

	def calculateRMS(self, sample):
	
		count = len(sample)
		data = np.array(sample).astype('float32')
		data *= (1.0 / 32768.0)
		return np.sqrt(np.mean(data**2.0))

	def detect_clap(self, block, time_now):
		data = np.asarray(block) #Convert mic data to numpy array

		#Flatten array and calculate RMS, average volume across all 4 mics
		all_data = data.flatten()
		sample_rms = self.calculateRMS(all_data)
		#print sample_rms, self.background_mean

		#Look for samples which exceed 10 time background noise
		if sample_rms > 10 * self.background_mean:
			self.over_threshold_count += 1 #If sample exceed volume, increment count
			self.clap_mean += sample_rms

		#When sample does not exceed threshold, test to see if number of consecutive samples which did is within clap limits
		else:
			# recent clap
			if self.over_threshold_count >= 1 and self.over_threshold_count < self.max_clap_samples:

				# update output
				self.blocks_since_clap = 0
				self.time_at_clap = time_now
				self.clap_seen = True
				self.clap_mean *= (1.0 / self.over_threshold_count)
				level = "{0:.3f}".format(self.clap_mean)
				bg = "{0:.3f}".format(self.background_mean)
				print "clap @", \
						"{0:.3f}".format(self.time_at_clap - self.t0), "sec", \
						", level =", level, ", background =", bg

			# no recent clap
			else:
				
				# update output
				self.blocks_since_clap += 1 #Increment timer

			# esimate background
			self.background_mean += self.background_lambda * (sample_rms - self.background_mean)

			# reset
			self.over_threshold_count = 0 #Reset count
			self.clap_mean = 0.0

