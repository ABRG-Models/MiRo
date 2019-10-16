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
import os
import copy

import node
import miro2 as miro

import cv2
from cv_bridge import CvBridge



class NodeDecode(node.Node):

	def __init__(self, sys):

		node.Node.__init__(self, sys, "decode")
		self.first_call = True

	def tick_camera(self, stream_index, msg):

		# perf
		self.perf.step('decode')

		# very occasionally, camera decoding can fail due to the JPEG
		# image being corrupt. not sure why this happens, but if it
		# does we handle it relatively silently
		try:
			img = cv2.imdecode(np.fromstring(msg.data, np.uint8), cv2.IMREAD_COLOR)
		except:
			# treat as if no image arrived
			print "JPEG decoding error - frame discarded"
			return 0

		# if no image at all, wait for one
		if img is None:
			return 0

		# always return -1 on the first call to force a camera
		# reconfiguration, which will fix the frame rate if it
		# not already correct
		if self.first_call:
			print "first call"
			self.first_call = False
			return -1

		# check image size - we are now only accepting the correct
		# image size, and we'll raise a flag to correct it if not
		SX = 640
		SY = 360
		if img.shape != (SY, SX, 3):
			print "image wrong size"
			return -1

		# intialize camera model
		if self.state.camera_model_full is None:
			self.state.camera_model_full = miro.utils.camera_model.CameraModel(self.pars)
			self.state.camera_model_full.set_frame_size_from_img(img)

		# store color frame
		self.state.frame_bgr_full[stream_index] = copy.copy(img)

		# store greyscale frame
		self.state.frame_gry_full[stream_index] = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

		# resize image to the size we process at in the demo
		des_image_height = self.pars.decode.image_height
		if img.shape[0] > des_image_height:
			sc = float(des_image_height) / img.shape[0]
			img = cv2.resize(img, dsize=(0, 0), fx=sc, fy=sc)

		# intialize camera model
		if self.state.camera_model_mini is None:
			self.state.camera_model_mini = miro.utils.camera_model.CameraModel(self.pars)
			self.state.camera_model_mini.set_frame_size_from_img(img)

		# store color frame
		self.state.frame_bgr[stream_index] = img

		# store greyscale frame
		self.state.frame_gry[stream_index] = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

		# perf
		self.perf.step('wait')

		# success
		return 1



