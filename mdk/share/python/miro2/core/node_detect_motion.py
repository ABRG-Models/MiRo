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

import cv2



class NodeDetectMotion(node.Node):

	def __init__(self, sys):

		node.Node.__init__(self, sys, "detect_motion")

		# state
		self.frame0 = [None, None]
		self.frame1 = [None, None]

	def tick_camera(self, stream_index, detect_motion):

		if detect_motion:

			self.frame1[stream_index] = self.frame0[stream_index]
			self.frame0[stream_index] = self.state.frame_bgr[stream_index]

			if self.frame1[stream_index] is not None:

				# take difference in RGB space
				dif = cv2.absdiff(self.frame0[stream_index], self.frame1[stream_index])

				# convert to grayscale
				mov = cv2.cvtColor(dif, cv2.COLOR_BGR2GRAY)

				# filter
				movf = cv2.GaussianBlur(mov, (9,9), 0)

				# publish
				self.state.frame_mov[stream_index] = movf


		else:

			# in this case, just provide a blank frame
			self.state.frame_mov[stream_index] = 0 * cv2.cvtColor(self.state.frame_bgr[stream_index], cv2.COLOR_BGR2GRAY)



