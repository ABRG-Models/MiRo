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
import copy
import multiprocessing

import cv2

import node

import miro2 as miro
import signals



class NodeSpatial(node.Node):

	def __init__(self, sys):

		node.Node.__init__(self, sys, "spatial")

		# resources
		self.lock = multiprocessing.Lock()

		# state
		self.jit_init_complete = False
		self.priority_peak = [
			signals.PriorityPeak(0),
			signals.PriorityPeak(1),
			signals.PriorityPeak(2)
		]

		# inputs
		self.audio_events = [[], [], []]
		self.wide_field_elev = 0.0
		self.face = [None, None]

		# state
		self.mru_eye = 0
		self.mov_gain = self.pars.spatial.mov_gain

		# debug
		if self.pars.flags.DEV_DEBUG_WRITE_TRACES:
			with open("/tmp/spatial", "w") as file:
				file.write("")

	def jit_init(self, stream_index, img_shape):

		# attempt to lock
		if not self.lock.acquire(False):

			# failed to lock, busy, so nothing we can do here
			print "jit_init busy in stream", stream_index, "(it is being run on the other camera stream)"
			return False

		# mutex is locked, now, check again if jit_init is already complete
		if self.jit_init_complete:

			# ok all done
			print "jit_init complete in stream", stream_index, "(already completed by the other camera stream when we got the mutex)"
			self.lock.release()
			return True

		# report
		print "jit_init started in stream", stream_index

		# recover dimensions from example image
		self.sx = img_shape[1]
		self.sy = img_shape[0]
		self.sw = 256

		# intialize camera model
		self.state.camera_model_mini = miro.utils.camera_model.CameraModel(self.pars)
		self.state.camera_model_mini.set_frame_size(self.sx, self.sy)

		# test camera model
		#print self.state.camera_model_mini.p2d([0.0, 0.0]).as_string()
		#miro.utils.error("stop")

		# initialize blank
		self.blank = np.zeros((self.sy, self.sx), np.float32)

		# initialize domes
		self.domes = []

		# initialize arrays for image streams
		self.pri  = [
			copy.copy(self.blank),
			copy.copy(self.blank),
			np.zeros((1, self.sw), np.float32)
		]

		# initialize wide field
		azim_max = (90.0 + self.pars.spatial.degrees_hindsight) * (np.pi / 180.0)
		azim_step = azim_max / (self.sw * 0.5)
		self.wide_azim = np.arange(azim_max-0.5*azim_step, -azim_max, -azim_step)

		# initialize camera field
		self.central_axis_azim = [None, None]
		self.central_axis_elev = [None, None]
		for stream_index in range(0, 2):

			central_axis_azim = np.zeros((self.sx))
			for xp in range(0, self.sx):
				p = [xp, self.sy * 0.5 - 0.5]
				v = self.state.camera_model_mini.p2v(p)
				central_axis_azim[xp] = v.azim + self.pars.camera.azimuth[stream_index]
			self.central_axis_azim[stream_index] = central_axis_azim

			central_axis_elev = np.zeros((self.sy))
			for yp in range(0, self.sy):
				p = [self.sx * 0.5 - 0.5, yp]
				v = self.state.camera_model_mini.p2v(p)
				central_axis_elev[yp] = v.elev + self.pars.camera.elevation[stream_index]
			self.central_axis_elev[stream_index] = central_axis_elev

		# report
		print "jit_init completed in stream", stream_index

		# mark completed
		self.jit_init_complete = True

		# release
		self.lock.release()

		# return completion state
		return True

	def get_dome(self, radius):

		# we store domes that we create, so we only have to create
		# them once at each radius
		for dome in self.domes:
			if dome[0] == radius:
				return dome[1]

		# create
		if not self.pars.flags.DEV_DEBUG_HALT: # do not pollute info
			print "create dome at radius", radius
		s = radius * 8 + 1
		dome = np.zeros((s, s), np.float32)

		# inject
		x = radius * 4
		dome[x, x] = 1.0

		# filter
		dome = cv2.GaussianBlur(dome, (0, 0), radius)

		# normalize
		dome *= (1.0 / np.max(dome))

		# store
		self.domes.append((radius, dome))

		# ok
		return dome

	def inject_pattern(self, frame, center, pattern):

		# get pattern size, assuming it is square
		sy = (pattern.shape[0] - 1) / 2
		sx = (pattern.shape[1] - 1) / 2

		# get source extent
		sx1 = 0
		sx2 = sx * 2 + 1
		sy1 = 0
		sy2 = sy * 2 + 1

		# get destination extent
		dx1 = center[0] - sx
		dx2 = center[0] + sx + 1
		dy1 = center[1] - sy
		dy2 = center[1] + sy + 1

		# constrain into destination
		sy = frame.shape[0]
		sx = frame.shape[1]
		if dx1 < 0:
			sx1 += (0 - dx1)
			dx1 = 0
		if dx2 > sx:
			sx2 += (sx - dx2)
			dx2 = sx
		if dy1 < 0:
			sy1 += (0 - dy1)
			dy1 = 0
		if dy2 > sy:
			sy2 += (sy - dy2)
			dy2 = sy

		# do injection
		frame[dy1:dy2, dx1:dx2] += pattern[sy1:sy2, sx1:sx2]

	def inject_dome(self, frame, center, radius, height):

		# simple in-fill of specified circle
		#cv2.circle(frame, center, radius, height, -1)

		# get dome
		dome = self.get_dome(radius)
		dome = dome * height

		# inject into frame
		self.inject_pattern(frame, center, dome)

	def estimate_range(self, size_in_pix, size_in_m):

		# first, convert size_in_pix to normalised image size
		size_norm = float(size_in_pix) / self.pars.decode.image_width

		# normalise that by known size of object
		size_rel = size_norm / size_in_m

		# we could then retrieve the range from theory, but rather
		# than bother to actually figure it out (it is somewhat
		# dependent on the camera distortion model) I'm just going
		# to estimate it empirically for now
		if size_rel > 0.0:
			range = 0.5 / size_rel
			#print "est range", size_in_pix, size_in_m, range
		else:
			range = self.pars.action.range_estimate_max

		if range < self.pars.action.range_estimate_min:
			range = self.pars.action.range_estimate_min
		if range > self.pars.action.range_estimate_max:
			range = self.pars.action.range_estimate_max

		# ok
		return range

	def inject_face(self, stream_index):

		# extract signal
		if self.state.detect_objects[stream_index] is None:
			return

		# for each face
		for face in self.state.detect_objects[stream_index].faces:

			# move coordinates into frame pixels
			conf = face.conf
			corn = self.state.camera_model_mini.d2p(np.array(face.corner)).astype('int32')
			size = self.state.camera_model_mini.length_d2p(np.array(face.size)).astype('int32')

			# get range
			range = self.estimate_range(size[0], self.pars.action.face_size_m)

			# debug
			if not self.pars.flags.DEV_DEBUG_HALT: # do not pollute info
				print "face at range", range, "with conf", conf
			if self.pars.flags.DEBUG_DETECTION:
				self.output.tone = 255

			# paint in face
			x = int(corn[0] + size[0] * 0.5)
			y = int(corn[1] + size[1] * 0.5)
			r = (size[0] + size[1]) * 0.25
			m = self.pars.spatial.face_gain * conf

			# choose radius that reflects representational size
			# based on physical size in image
			r = int(r * 0.5)

			# inject stimulus
			self.inject_dome(self.pri[stream_index], (x, y), r, m)

			# store source
			p = [x, y]
			vh = self.state.camera_model_mini.p2vh(p, stream_index)
			self.sources.append([self.pars.action.priority_source_index_face, vh, range])

	def inject_ball(self, stream_index):

		# extract signal
		if self.state.detect_objects[stream_index] is None:
			return

		# for each ball
		for ball in self.state.detect_objects[stream_index].balls:

			# move coordinates into frame pixels
			cen_p = self.state.camera_model_mini.d2p(np.array(ball.centre))
			rad_p = self.state.camera_model_mini.length_d2p(ball.radius)

			# get ball parameters
			x = int(cen_p[0])
			y = int(cen_p[1])
			r = int(rad_p)
			m = self.pars.spatial.ball_gain
			print x, y, r, m

			# get range
			range = self.estimate_range(r * 2, self.pars.action.ball_size_m)

			# debug
			if not self.pars.flags.DEV_DEBUG_HALT: # do not pollute info
				print "ball at range", range
			if self.pars.flags.DEBUG_DETECTION:
				self.output.tone = 253

			# choose radius that reflects representational size
			# based on physical size in image
			r = int(r * 0.5)

			# inject stimulus
			self.inject_dome(self.pri[stream_index], (x, y), r, m)

			# store source
			p = [x, y]
			vh = self.state.camera_model_mini.p2vh(p, stream_index)
			self.sources.append([self.pars.action.priority_source_index_ball, vh, range])

	def inject_april(self, stream_index):

		# extract signal
		if self.state.detect_objects[stream_index] is None:
			return

		# for each tag
		for tag in self.state.detect_objects[stream_index].tags:

			# extract tag
			id = tag.id
			cen = [tag.centre[0], tag.centre[1]]
			corn = [tag.corners[0:2], tag.corners[2:4], tag.corners[4:6], tag.corners[6:8]]

			# move coordinates into frame pixels
			cen = self.state.camera_model_mini.d2p(cen)
			c0 = self.state.camera_model_mini.d2p(np.array(corn[0]))
			c2 = self.state.camera_model_mini.d2p(np.array(corn[2]))

			# get size
			d = np.linalg.norm(c2 - c0)

			# get parameters
			x = int(cen[0])
			y = int(cen[1])
			r = int(d/2.0)

			# get range
			range = self.estimate_range(d * 0.707, self.pars.action.april_size_m)

			# compute magnitude
			gain = self.pars.spatial.april_gain_at_1m
			if range < 0.1:
				range = 0.1
			gain *= 1.0 / np.sqrt(range)
			gain = np.clip(gain, 0.0, 1.0)
			m = gain
			#print gain, range, m

			# inject stimulus
			self.inject_dome(self.pri[stream_index], (x, y), r, m)

			# debug
			if self.pars.flags.DEBUG_DETECTION:
				self.output.tone = 200

			# store source
			p = [x, y]
			vh = self.state.camera_model_mini.p2vh(p, stream_index)
			self.sources.append([self.pars.action.priority_source_index_april, vh, range])

	def inject_motion_debug(self, stream_index):

		if stream_index != 0:
			return

		frame_mov = self.state.frame_mov[stream_index].astype('float32') * (1.0 / 255.0)
		f = frame_mov - np.mean(frame_mov)
		x = np.max(f)
		with open("/tmp/mov", "a") as file:
			file.write(str(x) + " " + str(self.state.in_motion) + " " + str(self.mov_gain) + "\n")

	def inject_motion(self, stream_index):

		# if conditions are met
		if self.state.in_motion == 0 and self.state.in_blink == 0:

			# extract signal
			frame_mov = self.state.frame_mov[stream_index].astype('float32') * (1.0 / 255.0)

			# get de-meaned frame
			frame_mov_mean = np.mean(frame_mov)
			mov = frame_mov - frame_mov_mean

			# do adaptive threshold
			x = np.max(mov)
			if x > 0.0:
				tgt_gain = self.pars.spatial.mov_gain_adapt_tgt / x
				d = tgt_gain - self.mov_gain
				self.mov_gain += self.pars.spatial.mov_gain_adapt_gamma * d
				self.mov_gain = np.clip(self.mov_gain, self.pars.spatial.mov_gain_adapt_min, self.pars.spatial.mov_gain_adapt_max)

			# inject the de-meaned frame
			#print self.mov_gain
			self.pri[stream_index] += self.mov_gain * mov

	def inject_audio(self, stream_index):

		# extract and clear signal
		audio_events = self.audio_events[stream_index]
		self.audio_events[stream_index] = []

		# get gain
		gain = self.pars.spatial.audio_event_gain
		gain += self.state.in_making_noise * \
				(self.pars.spatial.audio_event_gain_making_noise - self.pars.spatial.audio_event_gain)

		# process audio events
		for audio_event in audio_events:

			# handle lr streams
			if stream_index < 2:

				# response in azim
				delta = audio_event.azim - self.central_axis_azim[stream_index]
				delta_sq = (delta * self.pars.spatial.audio_event_azim_size_recip) ** 2
				response_azim = np.exp(-delta_sq)

				# response in elev
				delta = audio_event.elev - self.central_axis_elev[stream_index]
				delta_sq = (delta * self.pars.spatial.audio_event_elev_size_recip) ** 2
				response_elev = np.exp(-delta_sq)

				# combine response
				response_azim = np.reshape(response_azim, (len(response_azim), 1))
				response_elev = np.reshape(response_elev, (len(response_elev), 1))
				response = np.dot(response_elev, response_azim.T)

			# handle wide stream
			if stream_index == 2:

				# response in azim
				delta = audio_event.azim - self.wide_azim
				delta_sq = (delta * self.pars.spatial.audio_event_azim_size_recip) ** 2
				response = np.exp(-delta_sq)

				# store elevation to be used in priority peak
				self.wide_field_elev = audio_event.elev

			# inject
			self.pri[stream_index] += (gain * audio_event.level) * response

	def publish_peak(self, peak):

		# attempt to lock
		if not self.lock.acquire(False):

			# failed to lock, someone else is doing it
			return

		# publish that
		self.state.priority_peak = peak

		# release
		self.lock.release()

	def find_best_peak(self):

		# extract peaks
		h = [
			self.priority_peak[0].height,
			self.priority_peak[1].height,
			self.priority_peak[2].height
		]

		# switching between orienting to the two eyes alternately
		# is problematic in a minor way because the cameras are
		# slightly mis-aligned so the focus is constantly switching.
		# to avoid this, we specify a slight tendency to stay with
		# the MRU eye.
		e = 1 - self.mru_eye
		h[e] *= 0.95

		# find best peak
		best_peak = 0
		for j in range(1, 3):
			if h[j] > h[best_peak]:
				best_peak = j

		# set mru_eye
		if best_peak < 2:
			self.mru_eye = best_peak

		# ok
		return self.priority_peak[best_peak]

	def compute_stream_peak(self, stream_index):

		# get image
		img = self.pri[stream_index]

		# find threshold for measuring size of priority region
		height = float(img.max())
		height_min = float(img.min())
		thresh = self.pars.spatial.pri_peak_height_thresh * height

		# handle special cases
		if height <= 0.0 or height_min >= thresh or height < 1e-4:
			N_accum = 0

		# handle normal case
		else:

			# find all points above threshold
			(y, x) = np.where(img > thresh)

			# describe region by centroid
			y_accum = sum(y)
			x_accum = sum(x)
			N_accum = len(x)

		# if nothing, we're done
		if N_accum == 0:

			# null peak
			peak = signals.PriorityPeak(stream_index)

		else:

			# centroid in pixel space
			p = [float(x_accum) / N_accum, float(y_accum) / N_accum]

			# handle camera streams
			if stream_index < 2:

				# convert to d space
				loc_d = self.state.camera_model_mini.p2d(p)

				# convert to view line referenced to head ("vh" in camera_model)
				vh = self.state.camera_model_mini.p2vh(p, stream_index)

				# size and height
				size = float(N_accum) / float(self.state.camera_model_mini.frame_pixel_count)

				# create peak
				peak = signals.PriorityPeak(stream_index, loc_d, height, size, vh.azim, vh.elev)

			# handle wide stream
			else:

				# size and height
				size = float(N_accum) / float(self.sw)

				# get azim and elev
				azim = self.wide_azim[int(p[0])]
				elev = self.wide_field_elev

				# create peak
				peak = signals.PriorityPeak(stream_index, None, height, size, azim, elev)

		# lay in, with protection
		with self.lock:
			self.priority_peak[stream_index] = peak

		# return peak
		return peak

	def process_stream(self, stream_index):

		# get our audio events
		q = self.state.audio_events_for_spatial
		self.state.audio_events_for_spatial = []

		# process audio events for all streams, regardless of
		# which stream we're processing right now; this is ok
		# because /any/ stream can do this, so long as it is
		# done as soon as the events are available from mics
		if len(q):
			for i in range(3):
				for event in q:
					self.audio_events[i].append(event)

		# do pri dynamics
		r = int(self.pars.decode.image_height * 0.15)
		prif = cv2.GaussianBlur(self.pri[stream_index], (r, r), 0)
		prif *= self.pars.spatial.pri_decay_lambda
		prif_mean = np.mean(prif)
		self.pri[stream_index] = prif - prif_mean

		# clear sources
		self.sources = []

		#DebugAdaptiveMotionThresh
		#self.inject_motion_debug(stream_index)

		# since servos do not complete their motion for a period after being
		# commanded, the configuration of the robot will not match the configuration
		# in kc_m for a period starting when a movement begins, and ending t_mov_settle
		# after the movement completes. if we continue injecting energy during this
		# period, we may begin a movement using false data (typically during the
		# settling period). one simple way to avoid this is to simply disallow any
		# injection during that period.
		#
		# NB: a more sophisticated way of handling this is to use a more timely model
		# of the body when interpreting the incoming data. this is what kc_s is intended
		# for. however, that requires some quite thorny processing - interpreting what
		# is in kc_s, but using it to drive a kc_m that is already ahead in time. this
		# is certainly possible, but will require some development. #SpeedyResponse
		if self.state.in_motion == 0.0:

			# for camera streams
			if stream_index < 2:

				# inject motion detector output
				if self.pars.flags.SALIENCE_FROM_MOTION:
					self.inject_motion(stream_index)

				# inject detected balls
				if self.pars.flags.SALIENCE_FROM_BALL:
					self.inject_ball(stream_index)

				# inject detected faces
				if self.pars.flags.SALIENCE_FROM_FACE:
					self.inject_face(stream_index)

				# inject detected tags
				if self.pars.flags.SALIENCE_FROM_APRIL:
					self.inject_april(stream_index)

			# inject sound events
			if self.pars.flags.SALIENCE_FROM_SOUND:
				self.inject_audio(stream_index)

		# compute and access priority peak for this stream
		peak = self.compute_stream_peak(stream_index)

		# review possible sources of this peak by comparing view lines of
		# each active source with the view line of the peak.
		#
		# NB: we are comparing sources we /just/ injected into this stream
		# with the peak of this stream, so it is safe to compare view lines
		# because they are all defined in the same coordinate space.
		for source in self.sources:

			# extract
			source_index = source[0]
			source_vh = source[1]
			source_range = source[2]

			# compare
			da = source_vh.azim - peak.azim
			de = source_vh.elev - peak.elev
			d = np.sqrt(da*da + de*de)

			# confidence that this individual source caused the overall peak
			conf = max(1.0 - d / self.pars.spatial.association_angle, 0.0)

			# load that source into peak
			value = self.pars.action.priority_source_appetitive_value[source_index]
			peak.append_source(conf, source_range, value)

		# find best peak across streams
		best_peak = self.find_best_peak()

		# finalize peak
		best_peak.finalize(self.pars)

		# publish best peak (now augmented with additional information)
		self.publish_peak(best_peak)

		# publish priority map (mostly for debug)
		x = self.pri[stream_index] * 255.0
		x = np.clip(x, 0, 255)
		self.state.frame_pri[stream_index] = x.astype(np.uint8)

		# for camera streams
		if stream_index < 2:

			# debug
			if self.pars.flags.DEV_DEBUG_WRITE_TRACES and stream_index == 0:

				if not self.state.detect_objects[stream_index] is None:

					# prepare string representing single tag and best_peak
					tags = self.state.detect_objects[stream_index].tags
					if len(tags) > 0:
						tag = tags[0]
						cen = tag.centre
					else:
						cen = [-1.0, -1.0]
					if best_peak.stream_index == 0:
						loc_d = best_peak.loc_d
					else:
						loc_d = [-1.0, -1.0]
					if loc_d is None:
						s_loc_d = "-1 -1"
					else:
						s_loc_d = str(loc_d[0]) + " " + str(loc_d[1])
					s_cen = str(cen[0]) + " " + str(cen[1])
					s = str(self.state.tick)
					s += " " + s_cen
					s += " " + s_loc_d
					s += " " + str(self.state.in_motion)

					# append to file
					with open("/tmp/spatial", "a") as file:
						file.write(s + "\n")

			# clear detected objects
			self.state.detect_objects[stream_index] = None

	def tick_camera(self, stream_index):

		# return list of streams updated
		updated = []

		# skip processing completely if motion processor not yet pipeline-full
		if self.state.frame_mov[stream_index] is None:
			return updated

		# ensure jit_init has been run
		if not self.jit_init_complete:
			if not self.jit_init(stream_index, self.state.frame_mov[stream_index].shape):
				return updated

		# this also triggers processing of that stream
		self.process_stream(stream_index)
		updated.append(stream_index)

		# stream 0 also triggers processing of the wide stream
		if stream_index == 0:
			self.process_stream(2)
			updated.append(2)

		# ok
		return updated



