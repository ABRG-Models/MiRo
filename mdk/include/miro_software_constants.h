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

/*
	This header provides constants used in the software.

	[0] by definition
	[1] measured by eye 181004 (BM)
	[2] was 5.0, but link was catching on servo horn securing screw, so
		limited travel to avoid this 181219 (BM)
	[3] was 1500, change necessitated by experiments 181219, not quite
		enough travel available to rearward end of movement
	[4] see dev/servo_lift_nonlinearity_map
	[5] was 1500, change necessitated by reduced gearing to allow full
		range of movement 181217
	[6] measured by eye 181217 (BM)
	[7] measured by eye 190201 (BM)
	[8] safely inside limits of servo travel 190201 (BM)
*/

#ifndef INC_MIRO_SOFTWARE_CONSTANTS_H
#define INC_MIRO_SOFTWARE_CONSTANTS_H



////////////////////////////////////////////////////////////////
//	WHEELS

//	main drive physical parameters
#define MIRO_WHEEL_DIAMETER_M				0.090f
#define MIRO_WHEEL_TRACK_M					0.164f
#define MIRO_WHEEL_MAX_SPEED_M_PER_S		0.400f

//	parameters derived from the above
#define MIRO_WHEEL_CIRCUMF_M				((3.14159f) * MIRO_WHEEL_DIAMETER_M)
#define MIRO_WHEEL_RADIUS_M					(MIRO_WHEEL_DIAMETER_M * 0.5f)
#define MIRO_WHEEL_HALF_TRACK_M				(MIRO_WHEEL_TRACK_M * 0.5f)
#define MIRO_WHEEL_MAX_ANG_SPEED_RAD_PER_S	(MIRO_WHEEL_MAX_SPEED_M_PER_S / MIRO_WHEEL_HALF_TRACK_M)

//	these additional parameters are used to convert between P1
//	signal space (normalised drive downstream and encoder counts
//	upstream) and P2 signal space (m/s both ways). for
//	applications working with the P2 signal space, these parameters
//	are not required, since the conversion is performed in P2.
#define MIRO_WHEEL_COUNTS_PER_MOTOR_ROT		4
#define MIRO_WHEEL_GEARBOX_RATIO			39.822f
#define MIRO_WHEEL_CMD_SPEED_PER_M_PER_S	210.0f

//	conversions between counts and distance
#define MIRO_WHEEL_COUNTS_PER_WHEEL_ROT (MIRO_WHEEL_COUNTS_PER_MOTOR_ROT * MIRO_WHEEL_GEARBOX_RATIO)
#define MIRO_WHEEL_M_PER_COUNT (MIRO_WHEEL_CIRCUMF_M / MIRO_WHEEL_COUNTS_PER_WHEEL_ROT)



////////////////////////////////////////////////////////////////
//	PHYSICAL SERVOS

#define MIRO_KIN_COUNTS_MIN						700 // [7]
#define MIRO_KIN_COUNTS_MAX						2300 // [7]

#define MIRO_COS_COUNTS_MIN						500 // tbc
#define MIRO_COS_COUNTS_MAX						2500 // tbc



////////////////////////////////////////////////////////////////
//	TILT

//	constants associated with TILT DOF (currently fixed).
#define MIRO_TILT_RAD_MIN						__DEG2RAD(   -5.0f)
#define MIRO_TILT_RAD_MAX						MIRO_TILT_RAD_MIN
#define MIRO_TILT_RAD_CALIB						MIRO_TILT_RAD_MIN



////////////////////////////////////////////////////////////////
//	LIFT

//	constants associated with LIFT DOF.
#define MIRO_LIFT_RAD_MIN						__DEG2RAD(   8.0f) // [2]
#define MIRO_LIFT_RAD_MAX						__DEG2RAD(   60.0f) // [1]
#define MIRO_LIFT_RAD_CALIB						__DEG2RAD(   34.0f) // [1]

#define MIRO_LIFT_COUNTS_CALIB					1600 // [3]

//	we actually use a nonlinear mapping for this DOF, below,
//	so the value here is not used.
#define MIRO_LIFT_COUNTS_PER_RAD				(660 / __DEG2RAD(30.0f))

/*c2py-skip*/
#define MIRO_LIFT_MAP_DEG2COUNTS (0.0050*x3+-0.6007*x2+39.7569*x1+737.4113) // [4]

/*c2py-skip*/
#define MIRO_LIFT_MAP_COUNTS2DEG (-0.0293*x3+1.4880*x2+-19.1194*x1+79.9066) // [4]




////////////////////////////////////////////////////////////////
//	YAW

//	Constants associated with YAW DOF.
#define MIRO_YAW_RAD_MIN					__DEG2RAD(-55.0f) // [8]
#define MIRO_YAW_RAD_MAX					__DEG2RAD(55.0f) // [8]
#define MIRO_YAW_RAD_CALIB					__DEG2RAD(0.0f) // [0]

#define MIRO_YAW_COUNTS_CALIB				1500 // [0]
#define MIRO_YAW_COUNTS_PER_RAD				(525 / __DEG2RAD(39.0f)) // [7]



////////////////////////////////////////////////////////////////
//	PITCH

//	Constants associated with PITCH DOF.
#define MIRO_PITCH_RAD_MIN					__DEG2RAD(  -22.0f) // [1]
#define MIRO_PITCH_RAD_MAX					__DEG2RAD(   8.0f) // [1]
#define MIRO_PITCH_RAD_CALIB				__DEG2RAD(    0.0f) // [0]

#define MIRO_PITCH_COUNTS_CALIB				1800 // [5]
#define MIRO_PITCH_COUNTS_PER_RAD			(770 / __DEG2RAD(16.8f)) // [6]



////////////////////////////////////////////////////////////////
//	TAIL

//	Constants associated with DROOP DOF.
#define MIRO_DROOP_CALIB				0.0f
#define MIRO_DROOP_DEFAULT				MIRO_DROOP_CALIB
#define MIRO_DROOP_COUNTS_CALIB			1500
#define MIRO_DROOP_COUNTS_0				1500
#define MIRO_DROOP_COUNTS_1				1370

//	Constants associated with WAG DOF.
#define MIRO_WAG_CALIB					0.5f
#define MIRO_WAG_DEFAULT				MIRO_WAG_CALIB
#define MIRO_WAG_COUNTS_CALIB			1500
#define MIRO_WAG_COUNTS_0				1300
#define MIRO_WAG_COUNTS_1				1700



////////////////////////////////////////////////////////////////
//	EYELIDS

//	Constants associated with EYELIDS DOFs.
//	"canonical" eye is the left one (for right, invert 0-calib and 1-calib)
#define MIRO_EYE_CALIB					0.5f
#define MIRO_EYE_DEFAULT				0.0f
#define MIRO_EYE_COUNTS_CALIB			1500
#define MIRO_EYE_COUNTS_0				1000
#define MIRO_EYE_COUNTS_1				2000



////////////////////////////////////////////////////////////////
//	EARS

//	Constants associated with EARS DOFs.
//	"canonical" ear is the left one (for right, invert 0-calib and 1-calib)
#define MIRO_EAR_CALIB					0.3333f
#define MIRO_EAR_DEFAULT				MIRO_EAR_CALIB
#define MIRO_EAR_COUNTS_CALIB			1500
#define MIRO_EAR_COUNTS_0				1750
#define MIRO_EAR_COUNTS_1				1000



////////////////////////////////////////////////////////////////
//	DIMENSIONS

//	location of TILT joint in FOOT
#define MIRO_LOC_TILT_X	(0.000f) // [1]
#define MIRO_LOC_TILT_Y	(0.000f) // [8]
#define MIRO_LOC_TILT_Z	(MIRO_WHEEL_DIAMETER_M * 0.5f) // [8]

//	location of LIFT joint in BODY
#define MIRO_LOC_LIFT_X (0.0022f) // [3]
#define MIRO_LOC_LIFT_Y (0.000f) // [1]
#define MIRO_LOC_LIFT_Z ( (0.0807f) - MIRO_LOC_TILT_Z ) // [3] (NB: inc. -MIRO_TILT_Z because specified figure is height above floor)

//	location of YAW joint in NECK
#define MIRO_LOC_YAW_X (0.000f) // [1]
#define MIRO_LOC_YAW_Y (0.000f) // [7]
#define MIRO_LOC_YAW_Z (0.1201f) // [3]

//	location of PITCH joint in GIMBAL
#define MIRO_LOC_PITCH_X (0.000f) // [1]
#define MIRO_LOC_PITCH_Y (0.000f) // [7]
#define MIRO_LOC_PITCH_Z (0.000f) // [7]

//	location of nose tip in HEAD
#define MIRO_LOC_NOSE_TIP_X (0.100f) // [3]
#define MIRO_LOC_NOSE_TIP_Y (0.000f) // [1]
#define MIRO_LOC_NOSE_TIP_Z (0.025f) // [3]

//	location of eye (camera optical centre) in HEAD ("canonical" eye is the left one: for right, invert y)
#define MIRO_LOC_EYE_X ( 0.0377f) // [3]
#define MIRO_LOC_EYE_Y ( 0.0431f) // [3]
#define MIRO_LOC_EYE_Z ( 0.0568f) // [3]

//	location of ear (microphone) in HEAD ("canonical" ear is the left one: for right, invert y)
#define MIRO_LOC_EAR_X (-0.0474f) // [3]
#define MIRO_LOC_EAR_Y ( 0.0519f) // [3]
#define MIRO_LOC_EAR_Z ( 0.0827f) // [3]

//	location of tail mic in BODY
#define MIRO_LOC_TAIL_MIC_X (-0.1600f)
#define MIRO_LOC_TAIL_MIC_Y ( 0.0000f)
#define MIRO_LOC_TAIL_MIC_Z ( (0.0850f) - MIRO_LOC_TILT_Z ) // (NB: inc. -MIRO_TILT_Z because specified figure is height above floor)

//	location of sonar fovea in HEAD
#define MIRO_LOC_SONAR_FOVEA_X (0.125f)
#define MIRO_LOC_SONAR_FOVEA_Y (0.000f)
#define MIRO_LOC_SONAR_FOVEA_Z (0.025f)

//	region sonar-local fovea can reach
#define MIRO_REACHABLE_Z_MIN 0.070f
#define MIRO_REACHABLE_Z_MAX 0.270f



////////////////////////////////////////////////////////////////
//	SENSORS

//	camera orientation in head
#define MIRO_CAM_ELEVATION __DEG2RAD( 45.0f) // [3]
#define MIRO_CAM_DIVERGENCE __DEG2RAD( 27.0f)

//	camera projection model (see dev/camera/projection)
#define MIRO_CAM_HORI_HALF_FOV __DEG2RAD(60.0f)
#define MIRO_CAM_PIXEL_ASPECT_RATIO 1.0f
#define MIRO_CAM_DISTORTION_MODEL_K1 -1.5f
#define MIRO_CAM_DISTORTION_MODEL_K2 1.0f
#define MIRO_CAM_DISTORTION_MODEL_K3 -0.5f
#define MIRO_CAM_DISTORTION_MODEL_K4 0.5f

//	IMU scaling
#define MIRO_ACCEL_1G (1024)

//	sonar scaling
#define MIRO_SONAR_READING_AT_1M (87.0f) // [10]



////////////////////////////////////////////////////////////////
//	FLAGS

//	DOCLINK PUSH FLAGS
//	body push types
#define MIRO_PUSH_FLAG_IMPULSE						__BIT(0)
#define MIRO_PUSH_FLAG_VELOCITY						__BIT(1)
#define MIRO_PUSH_FLAG_NO_TRANSLATION				__BIT(2)
#define MIRO_PUSH_FLAG_NO_ROTATION					__BIT(3)
#define MIRO_PUSH_FLAG_NO_NECK_MOVEMENT				__BIT(4)
#define MIRO_PUSH_FLAG_WAIT							__BIT(5)

//	DOCLINK AFFECT FLAGS
//	affect expression flags
#define MIRO_AFFECT_EXPRESS_THROUGH_VOICE			__BIT(0)
#define MIRO_AFFECT_EXPRESS_THROUGH_NECK			__BIT(1)
#define MIRO_AFFECT_EXPRESS_THROUGH_WHEELS			__BIT(2)



////////////////////////////////////////////////////////////////
//	IDENTIFIERS

//	link identifiers
#define MIRO_LINK_WORLD								-1
#define MIRO_LINK_FOOT								0
#define MIRO_LINK_BODY								1
#define MIRO_LINK_NECK								2
#define MIRO_LINK_GIMBAL							3
#define MIRO_LINK_HEAD								4
#define MIRO_LINK_COUNT								5

//	kin joint identifiers
#define MIRO_JOINT_TILT								0
#define MIRO_JOINT_LIFT								1
#define MIRO_JOINT_YAW								2
#define MIRO_JOINT_PITCH							3
#define MIRO_JOINT_COUNT							4

//	cos joint identifiers
#define MIRO_JOINT_DROOP							0
#define MIRO_JOINT_WAG								1
#define MIRO_JOINT_EYE_L							2
#define MIRO_JOINT_EYE_R							3
#define MIRO_JOINT_EAR_L							4
#define MIRO_JOINT_EAR_R							5

//	light sensor identifiers
#define MIRO_LIGHT_LF								0
#define MIRO_LIGHT_RF								1
#define MIRO_LIGHT_LR								2
#define MIRO_LIGHT_RR								3

//	cliff sensor identifiers
#define MIRO_CLIFF_L								0
#define MIRO_CLIFF_R								1

//	camera sensor identifiers
#define MIRO_CAM_L									0
#define MIRO_CAM_R									1

//	illum identifiers
#define MIRO_ILLUM_LF								0
#define MIRO_ILLUM_LM								1
#define MIRO_ILLUM_LR								2
#define MIRO_ILLUM_RF								3
#define MIRO_ILLUM_RM								4
#define MIRO_ILLUM_RR								5

//	tone message identifiers
#define MIRO_TONE_FREQ								0
#define MIRO_TONE_CODE								0
#define MIRO_TONE_VOL								1
#define MIRO_TONE_DUR								2



////////////////////////////////////////////////////////////////
//	CONTROL

#define MIRO_CLIFF_ADJ_MIN							-8
#define MIRO_CLIFF_ADJ_DEFAULT						0
#define MIRO_CLIFF_ADJ_MAX							8

#define MIRO_CLIFF_MEM_MIN							0
#define MIRO_CLIFF_MEM_DEFAULT						10
#define MIRO_CLIFF_MEM_MAX							50

#define MIRO_CLIFF_THRESH_MIN						0
#define MIRO_CLIFF_THRESH_DEFAULT					6
#define MIRO_CLIFF_THRESH_MAX						15

#define MIRO_CLIFF_MARGIN_MIN						0
#define MIRO_CLIFF_MARGIN_DEFAULT					1
#define MIRO_CLIFF_MARGIN_MAX						4



#endif




