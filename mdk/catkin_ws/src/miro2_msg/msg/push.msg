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
#	Description... TODO.

#	The identifier of the pushed link
int32 link

#	DOCLINK PUSH FLAGS
#
#	The values of these flags are defined in miro2.constants
#
#	PUSH_FLAG_IMPULSE
#		Treat push as an impulse (in mm).
#
#	PUSH_FLAG_VELOCITY
#		Treat push as a velocity (in mm/sec).
#
#	NB: If neither of the above flags is set, the push should
#	not be actioned at all (this condition is used in server
#	to indicate "not pending").
#
#	PUSH_FLAG_NO_TRANSLATION
#		Zero out any resultant change in pose.dr (i.e. only
#		turn on the spot).
#
#	PUSH_FLAG_NO_ROTATION
#		Zero out any resultant change in pose.dtheta /and/
#		in pose.dr (i.e. do not move wheels at all).
#
#	PUSH_FLAG_NO_NECK_MOVEMENT
#		Zero out any resultant change in neck configuration;
#		This flag is independent of NO_TRANSLATION/ROTATION.
#
#	PUSH_FLAG_WAIT
#		Cause the push processor to wait for further pushes
#		before publishing a velocity at its output. This is
#		required if you want to pass multiple push streams.
uint32 flags

#	The pushed point, in the reference frame of the pushed link
geometry_msgs/Point pushpos

#	The push vector, in the reference frame of the pushed link
geometry_msgs/Vector3 pushvec




