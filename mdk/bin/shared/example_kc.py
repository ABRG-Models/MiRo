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

import numpy as np
import miro2 as miro

# this example illustrates how to use the Kinematic Chain
# tool "kc" to transform positions and directions between
# the different frames of reference of the robot. the frames
# are listed in miro_constants.py.

# create kc object with default (calibration) configuration
# of joints (and zeroed pose of FOOT in WORLD)
kc = miro.utils.kc_interf.kc_miro()

# create objects in HEAD
pos = miro.utils.get("LOC_NOSE_TIP_HEAD")
vec = np.array([1.0, 0.0, 0.0])

# transform to WORLD (note use of "Abs" and "Rel"
# for positions and directions, respectively)
posw = kc.changeFrameAbs(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, pos)
vecw = kc.changeFrameRel(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, vec)

# report
print pos, vec
print posw, vecw

# update configuration based on data (imagine this came
# from /miro/sensors/kinematic_joints)
#
# NB: the immobile joint "TILT" is always at the same
# angle, "TILT_RAD_CALIB"
kinematic_joints = np.array([miro.constants.TILT_RAD_CALIB, np.radians(30.0), np.radians(15.0), np.radians(0.0)])
kc.setConfig(kinematic_joints)

# transform to WORLD
posw = kc.changeFrameAbs(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, pos)
vecw = kc.changeFrameRel(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, vec)

# report
print posw, vecw

