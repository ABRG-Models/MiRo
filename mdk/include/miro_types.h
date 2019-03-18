/**
	@section COPYRIGHT
	Copyright (C) 2019 Consequential Robotics Ltd
	
	@section AUTHOR
	Consequential Robotics http://consequentialrobotics.com
	
	@section LICENSE
	For a full copy of the license agreement, see LICENSE in the
	MDK root directory.
	
	Subject to the terms of this Agreement, Consequential
	Robotics grants to you a limited, non-exclusive, non-
	transferable license, without right to sub-license, to use
	MIRO Developer Kit in accordance with this Agreement and any
	other written agreement with Consequential Robotics.
	Consequential Robotics does not transfer the title of MIRO
	Developer Kit to you; the license granted to you is not a
	sale. This agreement is a binding legal agreement between
	Consequential Robotics and the purchasers or users of MIRO
	Developer Kit.
	
	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY
	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef INC_MIRO_TYPES_H
#define INC_MIRO_TYPES_H



//	force C linkage
#ifdef __cplusplus
extern "C" {
#endif

////////////////////////////////////////////////////////////////

//	pose in the plane (relative, or absolute in 2D WORLD)
struct MIRO_PLANAR_POSE
{
	float x;
	float y;
	float theta;
};

#define MIRO_PLANAR_POSE_SIZE 12

__PREPROC_ASSERT_SIZE(MIRO_PLANAR_POSE)

//	configuration of the body kinematic chain
struct MIRO_BODY_CONFIG
{
	float tilt;
	float lift;
	float yaw;
	float pitch;
};

#define MIRO_BODY_CONFIG_SIZE 16

__PREPROC_ASSERT_SIZE(MIRO_BODY_CONFIG)

//	complete state object for body
struct MIRO_BODY_STATE
{
	struct MIRO_PLANAR_POSE pose;
	struct MIRO_BODY_CONFIG config;
};

#define MIRO_BODY_STATE_SIZE 28

__PREPROC_ASSERT_SIZE(MIRO_BODY_STATE)

//	body velocity vector
struct MIRO_BODY_VEL
{
	float dr;		// mm/sec that axis centre moves forward
	float dtheta;	// rad/sec that BODY turns around axis centre
};

#define __MIRO_BODY_VEL_SET(vel, dr_, dtheta_) \
do { \
	vel.dr = dr_; \
	vel.dtheta = dtheta_; \
} while(0)

#define MIRO_BODY_VEL_SIZE 8

__PREPROC_ASSERT_SIZE(MIRO_BODY_VEL)

//	a line of view (typically, in MIRO's HEAD frame)
struct MIRO_VIEW_LINE
{
	float azim;			//	azimuth angle (Rad)
	float elev;			//	elevation angle (Rad)
};

#define MIRO_VIEW_LINE_SIZE 8

__PREPROC_ASSERT_SIZE(MIRO_VIEW_LINE)

//	generic primitive
struct MIRO_FLOAT2
{
	float x;
	float y;
};

#define MIRO_FLOAT2_SIZE 8

__PREPROC_ASSERT_SIZE(MIRO_FLOAT2)

//	generic primitive
struct MIRO_FLOAT3
{
	float x;
	float y;
	float z;
};

#define MIRO_FLOAT3_SIZE 12

__PREPROC_ASSERT_SIZE(MIRO_FLOAT3)

////////////////////////////////////////////////////////////////

//	force C linkage
#ifdef __cplusplus
}
#endif

#endif





