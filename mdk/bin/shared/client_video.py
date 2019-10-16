#!/usr/bin/python
#
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

import rospy
from sensor_msgs.msg import CompressedImage

import time
import sys
import os
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

# import april tags if available
try:
	import apriltag
	print "April Tags library imported OK"
except:
	print "April Tags library not available (use pip install apriltag)"



################################################################

def error(msg):
	print(msg)
	sys.exit(0)

################################################################

# if no argument provided
if len(sys.argv) == 1:

	# show usage
	print "pass one of the following arguments to set the mode:"
	print "\tshow: show video (eye cameras) as it arrives from platform"
	print "\trecord: record video to /tmp/client_video_*.avi"
	print "\noptionally include any of the following arguments:"
	print "\t--april: detect April Tags and annotate video"

	# done
	exit()
	
	
################################################################

class client:

	def callback_cam(self, ros_image, index):

		# silently (ish) handle corrupted JPEG frames
		try:

			# convert compressed ROS image to raw CV image
			image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "rgb8")

			# store image for display
			self.input_camera[index] = image

		except CvBridgeError as e:

			# swallow error, silently
			#print(e)
			pass

	def callback_caml(self, ros_image):

		self.callback_cam(ros_image, 0)

	def callback_camr(self, ros_image):

		self.callback_cam(ros_image, 1)
		
	def detect_april(self, im):
	
		# get grey frame
		im_grey = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

		# get all detected tags
		detected_tags = self.april_detector.detect(im_grey)
		
		# for each
		for tag in detected_tags:
		
			# accept only exact match
			if tag.hamming > 0:
				continue
				
			# annotate image
			corners = np.array(tag.corners).astype('int32')
			a = corners[3]
			for i in range(4):
				b = corners[i]
				cv2.line(im, (a[0], a[1]), (b[0], b[1]), (255, 0, 255), 2)
				a = b

		# ok	
		return im

	def loop(self):
	
		# state
		outfile = [None, None]
		outcount = [0, 0]
		t0 = time.time()
	
		# loop
		while not rospy.core.is_shutdown():
		
			# for each camera
			for index in range(2):
			
				# get image
				image = self.input_camera[index]
				
				# if present
				if not image is None:
				
					# handle
					self.input_camera[index] = None
					
					# april tags
					if not self.april_detector is None:
						image = self.detect_april(image)

					# if show
					if self.mode == "show":
						
						# show
						cv2.imshow("camera" + str(index), image)
						cv2.waitKey(1)

					# if record
					if self.mode == "record":
					
						# create output object
						if outfile[index] is None:
						
							# assume fps since we don't measure it
							fps = 15
						
							# create
							outfile[index] = cv2.VideoWriter( \
									'/tmp/client_video_' + str(index) + '.avi', \
									cv2.VideoWriter_fourcc('M','J','P','G'), \
									fps, (image.shape[1], image.shape[0]))
					
						# record
						outfile[index].write(image)
						
						# count
						outcount[index] += 1
						
						# report
						t1 = time.time()
						if (t1 - t0) > 1.0:
							t0 += 1.0
							print "frames so far", outcount

			# state
			time.sleep(0.02)

		# for each camera
		for index in range(2):
		
			# if open, release
			if not outfile[index] is None:
				outfile[index].release()

	def __init__(self, args):

		# state
		self.mode = None
		self.input_camera = [None, None]
		self.april_detector = None
		
		# handle arguments
		for arg in args:
			if arg in ["show", "record"]:
				self.mode = arg
				continue
			if arg == "--april":
				self.april_detector = True
				continue
			print "argument unrecognised:", arg
			exit()
		
		# check mode
		if self.mode is None:
			error("mode not set")

		# handle april
		if not self.april_detector is None:
			if "apriltag" not in sys.modules:
				raise ValueError("April Tags library not available")
			self.mode = self.mode.replace("-april", "")
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
			self.april_detector = apriltag.Detector(options)
		
		# ROS -> OpenCV converter
		self.image_converter = CvBridge()
		
		# robot name
		topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

		# subscribe
		self.sub_caml = rospy.Subscriber(topic_base_name + "/sensors/caml/compressed",
					CompressedImage, self.callback_caml, queue_size=1, tcp_nodelay=True)
		self.sub_camr = rospy.Subscriber(topic_base_name + "/sensors/camr/compressed",
					CompressedImage, self.callback_camr, queue_size=1, tcp_nodelay=True)
		
		# report
		print "recording from 2 cameras, press CTRL+C to halt..."

if __name__ == "__main__":

	rospy.init_node("client_video", anonymous=True)
	main = client(sys.argv[1:])
	main.loop()




