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

import numpy as np
import time

import node

import miro2 as miro

from node_detect_audio_engine import *



class NodeDetectAudio(node.Node):

	def __init__(self, sys):

		node.Node.__init__(self, sys, "detect_audio")

		# add extra pars
		self.pars.LINK_HEAD = miro.constants.LINK_HEAD
		self.pars.LINK_FOOT = miro.constants.LINK_FOOT

		# engine
		self.engine = DetectAudioEngine(self.pars, self.kc_s)

		# store
		self.store = []

	def tick_mics(self):

		msg = self.input.mics

		"""
		# for perftest
		self.store.append(msg.data)
		if len(self.store) == 100:
			file = open(r'mics_data', 'wb')
			pickle.dump(self.store, file)
			file.close()
			print "****"
		"""

		return self.engine.process_data(msg.data)



