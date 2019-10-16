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
# https://pypi.org/project/apriltag/

import numpy as np
import time
import apriltag
import cv2
import copy

import node
import miro2 as miro



class NodeDetectApril(node.Node):

	def __init__(self, sys):

		node.Node.__init__(self, sys, "detect_april")

		options = apriltag.DetectorOptions( \
				families='tag16h5',
				border=1,
				nthreads=4,
				quad_decimate=1.0,
				quad_blur=0.0,
				refine_edges=True,
				refine_decode=False,
				refine_pose=False,
				debug=False,
				quad_contours=True)

		self.detector = apriltag.Detector(options)

	def tick_camera(self, stream_index, msg):

		# get frame
		im = self.state.frame_gry_full[stream_index]

		# get all detected tags
		result = self.detector.detect(im)

		# pare by hamming distance
		if len(result) > 0:
			count = 0
			for i in range(len(result)):
				tag = result[i]

				id = tag[1]
				ham = tag[2]

				# pare by id and hamming distance
				if ham == 0 and id >= 1 and id <= 6:

					count += 1

					# extract fields
					family = tag[0]
					id = tag[1]
					goodness = tag[3]
					decmar = tag[4]
					hom = tag[5]
					cen = tag[6]
					corn = tag[7]

					# convert to d
					cen_d = self.state.camera_model_full.p2d(cen)
					corn_d = corn
					for i in range(4):
						corn_d[i] = self.state.camera_model_full.p2d(corn_d[i])

					# flatten
					corn_d = corn_d.flatten()

					# store
					tag = miro.msg.object_tag()
					tag.conf = goodness
					tag.id = id
					tag.centre = cen_d
					tag.corners = corn_d
					msg.tags.append(tag)



