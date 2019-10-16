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
#	This is the camera model computational code which is used
#	to transform visual information between reference frames.
#
#	COMMON IMAGE COORDINATES
#
#	All "normalised" image coordinates use the same scheme, as specified in camera_model.
#	In brief, we use x in [-0.5, 0.5] to indicate location of [left, right] edges of a
#	16:9 image of the same height as that in use. That is, if we are using a 4:3 image,
#	the edges of that image will actually be at [-0.375, 0.375]. This respects the physical
#	aspect ratio of the pixels, and the way the camera captures image data. It's also just
#	much easier to get your head round. Thus, the image parameter that dictates the scaling
#	is image height - but, the normalisation is to the width of a 16:9 image (so that a
#	16:9 image is one unit wide in x, and 9/16 high, whilst a 4:3 image is 3/4 units
#	wide in x and 9/16 high).
#
#	This scheme, "normalised" image coordinates, is denoted "d" in "camera_model.py".
#
#	See dev/camera/projection for details of derivation.
#
#	o OBJECT		three dimensional position
#					mm x, y, z
#
#		* pinhole (affine) projection
#
#	v VIEW LINE		azimuth & elevation in camera optics
#					Radians
#
#		* image space projection
#
#	u UNDISTORTED	vanilla projection in normalised coordinates
#					-0.5f to 0.5f (x, for 16:9 aspect, or -0.375f to 0.375f for 4:3)
#					-0.281f to 0.281f (y)
#
#		* barrel (lens) distortion
#
#	d DISTORTED		true projection in normalised coordinates
#					-0.5f to 0.5f (x, for 16:9 aspect, or -0.375f to 0.375f for 4:3)
#					-0.281f to 0.281f (y)
#
#		* image acquisition (including pixel aspect ratio)
#
#	p PIXEL			true projection in pixel coordinates
#					-0.5 to NX-0.5
#					-0.5 to NY-0.5
#					(note, then, that the bottom-left pixel at 0, 0 is not the edge of the image)
#
#	**** NB: p is not constrained to lie within the image! ****
#
#	We also define images in HEAD, which are:
#
#	oh				three dimensional position in HEAD
#					mm x, y, z
#
#	vh				view line from camera optical centre, but with azim/elev referenced to HEAD
#					Radians



import numpy as np
import copy
import miro2 as miro



class ViewLine(object):

	def __init__(self, azim=0.0, elev=0.0):
		self.azim = azim
		self.elev = elev

	def as_string(self):
		return "[{0:.5f},{1:.5f}]".format(self.azim, self.elev)



class CameraModel:

	def __init__( self, pars=None ):

		if pars is None:
			# use default if not supplied
			pars = miro.utils.platform_pars.PlatformPars()

		self.pars = pars.camera
		self.pars.pixel_aspect_ratio_recip = 1.0 / self.pars.pixel_aspect_ratio
		self.pars.v2u_scale = -0.5 / self.pars.hori_half_fov
		self.pars.u2v_scale = 1.0 / self.pars.v2u_scale

	def set_frame_size_from_img( self, img ):

		self.set_frame_size(img.shape[1], img.shape[0])

	def set_frame_size( self, x, y ):

		# we actually do not simply use the passed frame size. this would be
		# problematic, for one thing because we have derived a distortion model
		# based on a wide-screen (16:9) frame - if we use a 4:3 frame, we are
		# actually capturing from a smaller part of the sensor, and that model
		# would be wrong if we used it at the received frame width. in addition,
		# the scaling from pixels to view angles is easier if we stick with one
		# aspect ratio. thus, what we actually do is assume a 16:9 frame in this
		# model, with the caveat that we know that the centre pixel of the frame
		# at the size passed to this function will be at x/2, y/2, so we respect
		# that.
		#
		# in addition, our 180 high frames are actually 176 high. to avoid confusion,
		# we also promote these to 180 before deriving the model - but, again, we
		# respect the true image centre.

		# report
		print "set_frame_size of camera model to [" + str(x) + " x " + str(y) + "]"

		# compute location of centre of pixel image (p)
		self.p_cen_x = x / 2 - 0.5
		self.p_cen_y = y / 2 - 0.5

		# and compute pixel count
		self.frame_pixel_count = x * y

		# now co-erce 176 height to 180 because it's easier to follow and makes no odds
		if y == 176:
			y = 180

		# assume 16:9
		x = int(np.round(y * 16.0 / 9.0))
		print "using 16:9 frame size [" + str(x) + " x " + str(y) + "] to initialise camera model"

		# use assumed wide frame size to scale p <-> d
		self.d2p_scale = x
		self.p2d_scale = 1.0 / x

	def resolve_object(self, obj):

		ret = miro.signals.DetectObjectImage()
		ret.cx = int(obj.cx * self.d2p_scale + self.p_cen_x)
		ret.cy = int(obj.cy * self.d2p_scale + self.p_cen_y)
		ret.sx = int(obj.sx * self.d2p_scale)
		ret.sy = int(obj.sy * self.d2p_scale)
		return ret

	# apply distortion (map from affine projection space to image space).
	def u2d( self, u ):

		x_u = u[0]
		y_u = u[1]

		r = np.sqrt(x_u*x_u+y_u*y_u)
		z = 1.0 + self.pars.distortion_model_h1 * r
		x_i = z * x_u
		y_i = z * y_u

		q = x_i*x_i*y_i*y_i
		x_j = x_i * (1.0 - self.pars.distortion_model_h2 * q)
		y_j = y_i * (1.0 + self.pars.distortion_model_h3 * q)

		x_d = x_j * self.pars.distortion_model_h4
		y_d = y_j * self.pars.distortion_model_h4

		return [x_d, y_d]

	# apply distortion in reverse (map from image space to affine projection space).
	def d2u( self, d ):

		# first estimates are equal to input
		x_d = d[0]
		y_d = d[1]
		x_i = x_d
		y_i = y_d
		x_u = x_d
		y_u = y_d

		# iterate a fixed number of times to solve for input variables (no analytical solution)
		for iter in range(0, 4):

			# invert operation
			x_j = x_d / self.pars.distortion_model_h4
			y_j = y_d / self.pars.distortion_model_h4

			# invert operation
			q = x_i*x_i*y_i*y_i
			x_i = x_j / (1.0 - self.pars.distortion_model_h2 * q)
			y_i = y_j / (1.0 + self.pars.distortion_model_h3 * q)

			# invert operation
			r = np.sqrt(x_u*x_u+y_u*y_u)
			z = 1.0 + self.pars.distortion_model_h1 * r
			x_u = x_i / z
			y_u = y_i / z

		# ok
		return [x_u, y_u]

	# Map length from d (true) to p (pixel)
	def length_d2p( self, l_d ):

		return l_d * self.d2p_scale

	# Map length from p (pixel) to d (true)
	def length_p2d( self, l_p ):

		return l_p * self.p2d_scale

	# Map from d (true) to p (pixel).
	def d2p( self, d ):

		# create return value
		p = copy.copy(d)

		# apply pixel aspect ratio
		p[1] *= self.pars.pixel_aspect_ratio

		# scale by image width (yes, for y coordinate too - that's our standard)
		p[0] *= self.d2p_scale
		p[1] *= self.d2p_scale

		# transform by image centre
		p[0] += self.p_cen_x
		p[1] += self.p_cen_y

		# ok
		return p

	# Map from p (pixel) to d (true)
	def p2d( self, p ):

		# create return value
		d = copy.copy(p)

		# transform by image centre
		d[0] -= self.p_cen_x
		d[1] -= self.p_cen_y

		# scale by image width (yes, for y coordinate too - that's our standard)
		d[0] *= self.p2d_scale
		d[1] *= self.p2d_scale

		# apply pixel aspect ratio (inverse)
		d[1] *= self.pars.pixel_aspect_ratio_recip

		# ok
		return d

	# Map from p (pixel) to v (view line).
	def p2v( self, p ):

		# apply inverse image acquisition
		d = self.p2d( p )

		# apply inverse barrel distortion
		u = self.d2u( d )

		# recover view line from undistorted frame location
		#
		# the proper way to do this is to learn the mapping after the
		# inverse distortion model has been applied. this could be done
		# empirically, easily enough. however, informal tests show that
		# the mapping is pretty close to linear, so for expediency we
		# just use a linear model.
		azim = u[0] * self.pars.u2v_scale
		elev = u[1] * self.pars.u2v_scale
		v = ViewLine( azim, elev )
		#print self.p_cen_y, p[1], d[1], u[1], elev/np.pi*180.0

		# ok
		return v

	def v2vh(self, stream_index, v):

		vh = copy.copy(v)
		vh.azim += self.pars.azimuth[stream_index]
		vh.elev += self.pars.elevation[stream_index]
		return vh

	def p2vh(self, p, stream_index):

		v = self.p2v(p)
		vh = self.v2vh(stream_index, v)
		return vh

	def vh2oh(self, stream_index, vh, r):

		o = [r, 0.0, 0.0]
		o = miro.utils.kc.kc_rotate(o, 'z', vh.azim)
		o = miro.utils.kc.kc_rotate(o, 'y', -vh.elev)
		o += self.pars.location[stream_index]
		return o

	def v2oh(self, stream_index, v, r):

		vh = self.v2vh(stream_index, v)
		oh = self.vh2oh(stream_index, vh, r)
		return oh




