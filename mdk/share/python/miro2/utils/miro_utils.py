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
import os
import sys
import signal
import rospy
import threading
import time

import miro2 as miro



def cmd_vel2wheel_speed(dr, dtheta):

	# convert cmd_vel (dr in m/sec, dtheta in rad/sec) into wheel_speed
	a = dtheta * miro.constants.WHEEL_TRACK_M * 0.5
	l = dr - a
	r = dr + a

	# ok
	return [l, r]

def wheel_speed2cmd_vel(wheel_speed):

	# convert wheel_speed ([l, r] in m/sec) into cmd_vel
	dr = (wheel_speed[0] + wheel_speed[1]) / 2.0
	dtheta = (wheel_speed[1] - wheel_speed[0]) / miro.constants.WHEEL_TRACK_M

	# ok
	return (dr, dtheta)

def get(key):

	c = miro.constants

	if key == "LOC_NOSE_TIP_HEAD":
		return np.array([c.LOC_NOSE_TIP_X, c.LOC_NOSE_TIP_Y, c.LOC_NOSE_TIP_Z])

	if key == "LOC_EYE_L_HEAD":
		return np.array([c.LOC_EYE_X, c.LOC_EYE_Y, c.LOC_EYE_Z])

	if key == "LOC_EYE_R_HEAD":
		return np.array([c.LOC_EYE_X, -c.LOC_EYE_Y, c.LOC_EYE_Z])

	if key == "LOC_EAR_L_HEAD":
		return np.array([c.LOC_EAR_X, c.LOC_EAR_Y, c.LOC_EAR_Z])

	if key == "LOC_EAR_R_HEAD":
		return np.array([c.LOC_EAR_X, -c.LOC_EAR_Y, c.LOC_EAR_Z])

	if key == "LOC_TAIL_MIC_HEAD":
		return np.array([c.LOC_TAIL_MIC_X, c.LOC_TAIL_MIC_Y, c.LOC_TAIL_MIC_Z])

	if key == "LOC_SONAR_FOVEA_HEAD":
		return np.array([c.LOC_SONAR_FOVEA_X, -c.LOC_SONAR_FOVEA_Y, c.LOC_SONAR_FOVEA_Z])

	miro.utils.error("unknown key")

def get_media_file(filename):

	# since R190828, we keep multiple MDKs alongside; to avoid repeating some
	# large files, we do not keep them in all MDKs - rather, we just update
	# them if they change, or if we add any. here, we find the most recent
	# version of a given file for the caller.

	# media source list
	DIR_SOURCE = []

	# root of MDKs on-board
	DIR_CHANNEL = os.getenv("HOME") + "/channel"

	# if present (i.e. if running on-board)
	if os.path.isdir(DIR_CHANNEL):

		# list directories belonging to other releases
		DIR_MDK = []
		subdirs = os.listdir(DIR_CHANNEL)
		for d in subdirs:
			if len(d) < 3:
				continue
			if d[0:3] != "mdk":
				continue
			DIR_MDK.append(d)

		# rsort them so we prioritise more recent ones
		DIR_MDK.sort()
		DIR_MDK.reverse()

		# and append to media source list
		for d in DIR_MDK:
			DIR_SOURCE.append(DIR_CHANNEL + "/" + d + "/share/media")

	# append dev directories
	DIR_SOURCE.append(os.getenv('HOME') + "/mdk/share/media")
	DIR_SOURCE.append(os.getenv('HOME') + "/lib/miro2/mdk/share/media")
	DIR_SOURCE.append(os.getenv('HOME') + "/lib/miro2x/mdk/share/media")

	# now search them in order for the required file
	for dir in DIR_SOURCE:
		path = dir + "/" + filename
		if os.path.isfile(path):
			return path

	# not found
	return None
