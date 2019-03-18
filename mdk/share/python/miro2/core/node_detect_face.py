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
import os
import copy

import node

import cv2
from cv_bridge import CvBridge



class NodeDetectFace(node.Node):

	def __init__(self, sys):

		node.Node.__init__(self, sys, "detect_face")

		# resources
		self.bridge = CvBridge()

		# classifiers
		classifiers = [
				'haarcascade_frontalface_default.xml',
				#'haarcascade_frontalface_alt.xml',
				#'haarcascade_profileface.xml'
				]

		# initialise classifiers -- should be a list of haar-cascade xmls
		home = os.getenv("HOME")
		dir_src = home + "/mdk/share/data"
		if not os.path.isfile(dir_src + "/" + classifiers[0]):
			dir_src = home + "/lib/miro2x/mdk/share/data"
		self.cascades = [cv2.CascadeClassifier(dir_src + '/' + c) for c in classifiers]

		# clock state
		self.ticks = [0, 0]

	def tick_camera(self, stream_index):

		# get image (grayscale)
		img = self.state.frame_gry[stream_index]

		# load test image
		if self.pars.flags.DEV_DETECT_FACE and stream_index == 0 and (self.ticks[stream_index] & 30) < 1:
			home = os.getenv("HOME")
			img_color = cv2.imread(home + "/lib/miro2x/resource/detect_face_test.jpg")
			img = cv2.cvtColor(img_color, cv2.COLOR_BGR2GRAY)

		# search image with each classifier
		faces = []
		for cascade in self.cascades:
			f = cascade.detectMultiScale3(img, 1.05, 3, 0, (20, 20), outputRejectLevels=True)
			rects = f[0]
			confs = f[2]
			#rects = np.array([[ 20,  17,  27,  27], [107,  26,  38,  38]])
			#confs = np.array([[-1.18725086], [ 3.81156203]])
			for i in range(len(rects)):
				conf = confs[i]
				rect = rects[i].astype('float')
				conf = np.tanh(conf * 0.1)
				if conf > 0.0:
					#print "detected face", stream_index, conf, rect
					face = np.concatenate((rect, conf))
					faces.append(face)

		# merge duplicates
		# let's not do this for now, I'm not sure why it's needed yet
		#faces = self.merge_duplicates(faces)

		# store
		self.state.detect_face[stream_index] = faces

		# tick
		self.ticks[stream_index] += 1

	def merge_duplicates(self, faces):

		faces_out = copy.copy(faces)
		num_faces = len(faces)

		"""
		a = copy.copy(faces)
		overlap = False
		"""

		# check for any overlapping rectangles
		for i in range(0,num_faces):
			for j in range(i+1,num_faces):

				(xi, yi, wi, hi) = faces[i]
				(xj, yj, wj, hj) = faces[j]

				# conditions for overlap
				# if any of these are true, no overlap
				c1 = xj + wj < xi
				c2 = xi + wi < xj
				c3 = yj + hj < yi
				c4 = yi + hi < yj

				if c1 or c2 or c3 or c4:
					continue
				else:
					# we have an overlapping rectangle (face)
					# combine the two into one rectangle
					x = min(xi, xj)
					y = min(yi, yj)
					xb = max(xi + wi, xj + wj)
					yb = max(yi + hi, yj + hj)
					w = xb - x
					h = yb - y

					# append this to faces_out
					faces_out.append([x,y,w,h])

					# and remove the old ones
					try:
						faces_out.remove(faces[i])
					except:
						pass
					try:
						faces_out.remove(faces[j])
					except:
						pass

					# now replace the rectangle in faces with the larger
					# rectangle for future overlap checks
					faces[i] = [x,y,w,h]
					faces[j] = [x,y,w,h]

		"""
		b = copy.copy(faces_out)
		if overlap:
			print "overlap", num_faces, len(b)
			print a
			print b
		"""

		# ok
		return faces_out

		"""
		for now, we process all faces, not just the largest...

	def get_rectangle_area(self, rect):

		(x,y,w,h) = rect
		return w * h

	def get_largest_face(self, faces):

		if len(faces) < 1:
			return []

		# Find largest area
		areas = []
		for face in faces:
			areas.append(self.get_rectangle_area(face))

		return faces[np.argmax(areas)]

	def process(self, stream_index):

		# output message
		msg = self.pub_face.msg

		# search image for face
		img = self.img[stream_index]
		faces = self.search_img(img)

		# get largest face detected
		largest = self.get_largest_face(faces)

		# if nothing detected
		if len(largest) == 0:
			return
		"""

