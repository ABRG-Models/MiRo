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
#import pickle

import node
import miro2 as miro

import cv2



class NodeDetectBall(node.Node):

	def __init__(self, sys):

		node.Node.__init__(self, sys, "detect_ball")

		# clock state
		self.ticks = [0, 0]

	def tick_camera(self, stream_index, msg_obj):

		# get image from stream
		img = self.state.frame_bgr[stream_index]

		# load test image
		if self.pars.dev.DETECT_BALL and stream_index == 0 and (self.ticks[stream_index] & 30) < 1:
			home = os.getenv("HOME")
			img = cv2.imread(home + "/lib/miro2x/resource/detect_ball_test.jpg")

		# Get HSV
		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

		# red ball doesn't work as well because of skin, so
		# we use blue ball by default. for red, we cross the
		# hue boundary in hsv, so we have to use two masks
		#
		# (red 1)
		# lo = np.array([  0,  70,  70])
		# hi = np.array([ 10, 255, 255])
		#
		# (red 2)
		# lo = np.array([170,  70,  70])
		# hi = np.array([180, 255, 255])

		# get blue mask
		# blue ~ 90-150
		# green ~ 30-90
		# red ~ 150-30
		lo = np.array([90,  80,  80])
		hi = np.array([150, 255, 255])
		mask = cv2.inRange(hsv, lo, hi)

		# dump example for use with dev/ball_detect
		#with open('/tmp/ball', 'wb') as f:
		#	pickle.dump(mask, f)

		# clean up
		seg = mask
		seg = cv2.GaussianBlur(seg, (5, 5), 0)
		seg = cv2.erode(seg, None, iterations=2)
		seg = cv2.dilate(seg, None, iterations=2)

		# parameters
		canny_high_thresh = 128 # don't think it matters much for binary image, but does affect our grey image
		ball_detect_sensitivity = 20 # was 33 in Tom's code, lower detects more circles, so it's a trade-off
		ball_detect_min_dist_between_cens = 40 # was 40 in Tom's code, arbitrary
		ball_detect_min_radius = 5 # arbitrary, empirical, too small and we'll pick up noise objects
		ball_detect_max_radius = 60 # arbitrary, empirical

		# get circles
		circles = cv2.HoughCircles(seg, cv2.HOUGH_GRADIENT,
				1, ball_detect_min_dist_between_cens, \
				param1=canny_high_thresh, param2=ball_detect_sensitivity, \
				minRadius=ball_detect_min_radius, maxRadius=ball_detect_max_radius)

		# get largest circle
		ball = None
		if circles is not None:
			self.max_rad = 0
			circles = np.uint16(np.around(circles))

			for c in circles[0,:]:
				#cv2.circle(seg, (c[0], c[1]), c[2], (0, 255, 0), 2)

				if c[2] > self.max_rad:
					self.max_rad = c[2]
					ball = c

		# store detection
		if not ball is None:

			# convert to d
			cen_d = self.state.camera_model_mini.p2d(np.array(ball[0:2]).astype('float32'))
			rad_d = self.state.camera_model_mini.length_p2d(np.array(ball[2:]).astype('float32'))

			# create message
			msg = miro.msg.object_ball()
			msg.conf = 1.0
			msg.centre = cen_d
			msg.radius = rad_d

			# send
			msg_obj.balls.append(msg)

		# report detection
		if not ball is None:
			print "detect ball", stream_index, ball

		# publish for debug
		self.state.frame_bal[stream_index] = seg

		# tick
		self.ticks[stream_index] += 1
