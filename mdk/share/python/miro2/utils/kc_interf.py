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

# this file provides the interface between the generic kc
# object and the system that is using it

import kc
import miro2 as miro
import numpy as np

KC_FRAME_HEAD=4

kc.kc_init(miro.constants.PLATFORM_TICK_SEC, kc.KC_PUSH_FLAG_VELOCITY, KC_FRAME_HEAD)

def kc_miro():

	c = miro.constants

	return kc.KinematicChain([
			['FOOT',
				np.array([0.0, 0.0, 0.0]),
				'z',
				0.0,
				[kc.KC_ANGLE_UNCONSTRAINED, kc.KC_ANGLE_UNCONSTRAINED, kc.KC_ANGLE_UNCONSTRAINED] ],
			['BODY',
				np.array([c.LOC_TILT_X, c.LOC_TILT_Y, c.LOC_TILT_Z]),
				'y',
				c.TILT_RAD_CALIB,
				[c.TILT_RAD_CALIB, c.TILT_RAD_CALIB, c.TILT_RAD_CALIB] ],
			['NECK',
				np.array([c.LOC_LIFT_X, c.LOC_LIFT_Y, c.LOC_LIFT_Z]),
				'y',
				c.LIFT_RAD_CALIB,
				[c.LIFT_RAD_MIN, c.LIFT_RAD_MAX, kc.KC_ANGLE_UNCONSTRAINED] ],
			['GMBL',
				np.array([c.LOC_YAW_X, c.LOC_YAW_Y, c.LOC_YAW_Z]),
				'z',
				c.YAW_RAD_CALIB,
				[c.YAW_RAD_MIN, c.YAW_RAD_MAX, kc.KC_ANGLE_UNCONSTRAINED] ],
			['HEAD',
				np.array([c.LOC_PITCH_X, c.LOC_PITCH_Y, c.LOC_PITCH_Z]),
				'y',
				c.PITCH_RAD_CALIB,
				[c.PITCH_RAD_MIN, c.PITCH_RAD_MAX, kc.KC_ANGLE_UNCONSTRAINED] ],
		])



def kc_view_to_HEAD(azim, elev, r):

	# compute target
	target = np.array([r, 0.0, 0.0])

	# rotate target by elevation then azimuth
	target = kc.kc_rotate(target, 'y', -elev)
	target = kc.kc_rotate(target, 'z', azim)

	# ok
	return target




