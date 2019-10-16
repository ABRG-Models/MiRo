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

import time
import os
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image

class Vision:
	def __init__(self):

		self.image_converter = CvBridge()

		self.cam_images = [None, None]
		self.ann_images = [None, None]
		self.frame_w = None
		self.frame_h = None
		self.x_cent = None
		self.y_cent = None

		self.found_circle = [None, None]
		self.circle_str = [None, None]

		self.topic_root = '/' + os.getenv("MIRO_ROBOT_NAME") + '/'
		self.ann_image_pub_l = rospy.Publisher(self.topic_root + "sensors/cam_l/annotated", Image, queue_size=0)
		self.ann_image_pub_r = rospy.Publisher(self.topic_root + "sensors/cam_r/annotated", Image, queue_size=0)


	def process(self, img_data, index):
		try:
			cam_image = self.image_converter.compressed_imgmsg_to_cv2(img_data, "bgr8")
		except CvBridgeError as e:
			print("Conversion of image failed \n")
			print(e)

		if self.frame_w is None:
			im_h, im_w = cam_image.shape[:2]
			self.frame_w, self.frame_h = im_w, im_h
			self.x_cent = self.frame_w / 2.0
			self.y_cent = self.frame_h / 2.0

		self.cam_images[index] = cam_image

		# currently we don't do annotate
		return

		self.annotate(index)
		self.publish(index)

	def annotate(self, index):
		self.ann_images[index] = self.cam_images[index].copy()
		cv2.line(self.ann_images[index], (int(round(self.x_cent)), 0), (int(round(self.x_cent)), self.frame_h), (0, 0, 0), 4)
		cv2.line(self.ann_images[index], (0, int(round(self.y_cent))), (self.frame_w, int(round(self.y_cent))), (0, 0, 0), 4)
		cv2.line(self.ann_images[index], (int(round(self.x_cent)), 0), (int(round(self.x_cent)), self.frame_h), (255, 255, 255), 2)
		cv2.line(self.ann_images[index], (0, int(round(self.y_cent))), (self.frame_w, int(round(self.y_cent))), (255, 255, 255), 2)

		if not self.found_circle[index] is None:
			cv2.circle(self.ann_images[index], (self.found_circle[index][0], self.found_circle[index][1]), self.found_circle[index][2], (0, 255, 0), 2)
			cv2.circle(self.ann_images[index], (self.found_circle[index][0], self.found_circle[index][1]), 1, (0, 255, 0), 2)

			text_y_offset = 22
			for i, line in enumerate(self.circle_str[index].split(",")):
				text_y = self.found_circle[index][1] - text_y_offset + i*text_y_offset
				cv2.putText(self.ann_images[index], line, (self.found_circle[index][0]+5, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 6)
				cv2.putText(self.ann_images[index], line, (self.found_circle[index][0]+5, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

	def clear(self, index):
		self.found_circle[index] = None

	def publish(self, index):
		if index == 0:
			self.ann_image_pub_l.publish(self.image_converter.cv2_to_imgmsg(self.ann_images[0], "bgr8"))
		else:
			self.ann_image_pub_r.publish(self.image_converter.cv2_to_imgmsg(self.ann_images[1], "bgr8"))

	def detect_ball(self, colour_str, index):

		# get image
		im = self.cam_images[index]
		if im is None:
			return None

		# get image in HSV format
		im_hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

		#create colour code from user selected colour
		red = int(colour_str[1:3], 16)
		green = int(colour_str[3:5], 16)
		blue = int(colour_str[5:7], 16)
		bgr_colour = np.uint8([[[blue, green, red]]])
		hsv_colour = cv2.cvtColor(bgr_colour, cv2.COLOR_BGR2HSV)
		hue0 = hsv_colour[0,0][0]

		# define hue range
		huer = 30

		# mask image
		hue_min = hue0 - huer
		hue_max = hue0 + huer
		if hue_min < 0:
			lo = np.array([0, 70, 70])
			hi = np.array([hue_max, 255, 255])
			mask1 = cv2.inRange(im_hsv, lo, hi)
			lo = np.array([hue_min + 180, 70, 70])
			hi = np.array([180, 255, 255])
			mask2 = cv2.inRange(im_hsv, lo, hi)
			mask = cv2.bitwise_or(mask1, mask2)
		elif hue_max > 180:
			lo = np.array([hue_min, 70, 70])
			hi = np.array([180, 255, 255])
			mask1 = cv2.inRange(im_hsv, lo, hi)
			lo = np.array([0, 70, 70])
			hi = np.array([hue_max-180, 255, 255])
			mask2 = cv2.inRange(im_hsv, lo, hi)
			mask = cv2.bitwise_or(mask1, mask2)
		else:
			lo = np.array([hue_min, 70, 70])
			hi = np.array([hue_max, 255, 255])
			mask = cv2.inRange(im_hsv, lo, hi)

		# debug
		#cv2.imshow('im', mask)
		#cv2.waitKey(1)

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
				minRadius=ball_detect_min_radius, maxRadius=0)

		# Get largest circle
		max_circle = None
		#max_circle_norm = [None, None, None]
		if circles is not None:
			self.max_rad = 0
			circles = np.uint16(np.around(circles))

			for c in circles[0,:]:

				# annotate
				#cv2.circle(seg, (c[0], c[1]), c[2], (0, 255, 0), 2)

				if c[2] > self.max_rad:
					self.max_rad = c[2]
					max_circle = c

					"""
					max_circle_norm[0] = int(round(((max_circle[0] - self.x_cent) / self.x_cent) * 100.0))
					max_circle_norm[1] = int(round(-((max_circle[1] - self.y_cent) / self.y_cent) * 100.0))
					max_circle_norm[2] = int(round((max_circle[2]/self.x_cent)*100.0))
					"""

			self.found_circle[index] = max_circle
			#self.circle_str[index] = "x: " + str(max_circle_norm[0]) + "," + "y: " + str(max_circle_norm[1]) + "," + "r: " + str(max_circle_norm[2])

			# normalise
			if not max_circle is None:
				max_circle = np.array(max_circle).astype('float32')
				max_circle[0] -= self.x_cent
				max_circle[0] /= self.frame_w
				max_circle[1] -= self.y_cent
				max_circle[1] /= self.frame_w
				max_circle[1] *= -1.0
				max_circle[2] /= self.frame_w

				# convert to list
				m = max_circle
				return [m[0], m[1], m[2]]

			return None

		else:
			return None
