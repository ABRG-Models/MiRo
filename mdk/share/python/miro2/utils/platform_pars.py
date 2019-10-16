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

import miro2 as miro
import numpy as np



class TimingPars(object):

	def __init__(self):

		self.tick_hz = miro.constants.PLATFORM_TICK_HZ
		self.tick_sec = miro.constants.PLATFORM_TICK_SEC



class GeomPars(object):

	def __init__(self):

		self.reachable_z_min = miro.constants.REACHABLE_Z_MIN
		self.reachable_z_max = miro.constants.REACHABLE_Z_MAX



class CameraPars(object):

	def __init__(self):

		self.elevation = [miro.constants.CAM_ELEVATION, miro.constants.CAM_ELEVATION]
		self.azimuth = [miro.constants.CAM_DIVERGENCE, -miro.constants.CAM_DIVERGENCE]
		self.location = [
			[miro.constants.LOC_EYE_X, miro.constants.LOC_EYE_Y, miro.constants.LOC_EYE_Z],
			[miro.constants.LOC_EYE_X, -miro.constants.LOC_EYE_Y, miro.constants.LOC_EYE_Z]
			]
		self.hori_half_fov = miro.constants.CAM_HORI_HALF_FOV
		self.vert_half_fov = miro.constants.CAM_VERT_HALF_FOV
		self.tan_hori_half_fov = np.tan(self.hori_half_fov)
		self.norm_focal_length = 0.5 / self.tan_hori_half_fov
		self.pixel_aspect_ratio = miro.constants.CAM_PIXEL_ASPECT_RATIO
		self.distortion_model_h1 = miro.constants.CAM_DISTORTION_MODEL_H1
		self.distortion_model_h2 = miro.constants.CAM_DISTORTION_MODEL_H2
		self.distortion_model_h3 = miro.constants.CAM_DISTORTION_MODEL_H3
		self.distortion_model_h4 = miro.constants.CAM_DISTORTION_MODEL_H4



class PlatformPars (object):

	def __init__(self):

		self.timing = TimingPars()
		self.geom = GeomPars()
		self.camera = CameraPars()




