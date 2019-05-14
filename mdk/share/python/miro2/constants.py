

def __BIT(b):

	return 1 << b



def __DEG2RAD(x):

	return x * 0.0174532









# warning ignored line: #define INC_MIRO_FIRMWARE_CONSTANTS_H



################################################################
#	TIME

#	MIRO platform internal tick time
PLATFORM_TICK_HZ = (50)
PLATFORM_TICK_SEC = ((1.0) / PLATFORM_TICK_HZ)
PLATFORM_TICK_USEC = (1000000 / PLATFORM_TICK_HZ)

#	MIRO microsecond timer rolls over at a human-readable point
TIMER_ROLLOVER_SEC = (1000)
TIMER_ROLLOVER_USEC = (TIMER_ROLLOVER_SEC * 1000 * 1000)

#	audio sample rates
MIC_SAMPLE_RATE = 20000
SPKR_SAMPLE_RATE = 8000



################################################################
#	FLAGS

#	(non-persistent) downstream flags (per-tick)
PLATFORM_D_FLAG_DISABLE_ILLUM = __BIT(0)
PLATFORM_D_FLAG_DISABLE_STATUS_LEDS = __BIT(1)
PLATFORM_D_FLAG_DISABLE_SONAR = __BIT(2)
PLATFORM_D_FLAG_DISABLE_CLIFF = __BIT(3)
PLATFORM_D_FLAG_DISABLE_TOUCH_HEAD = __BIT(4)
PLATFORM_D_FLAG_DISABLE_TOUCH_BODY = __BIT(5)
PLATFORM_D_FLAG_DISABLE_CAMERAS = __BIT(6)
PLATFORM_D_FLAG_DISABLE_BODY_MICS = __BIT(7)
PLATFORM_D_FLAG_DISABLE_HEAD_MICS = __BIT(8)
PLATFORM_D_FLAG_DISABLE_SPEAKER = __BIT(9)
PLATFORM_D_FLAG_DISABLE_SPI_USR = __BIT(10)
PLATFORM_D_FLAG_DISABLE_DOMAIN_REPORTS = __BIT(11)
PLATFORM_D_FLAG_DISABLE_WHEEL_CONTROL = __BIT(12)
PLATFORM_D_FLAG_DISABLE_OPTO = __BIT(13)
PLATFORM_D_FLAG_DISABLE_SERVO_POWER = __BIT(14)
PLATFORM_D_FLAG_DISABLE_SERVO_SENSORS = __BIT(15)
PLATFORM_D_FLAG_DISABLE_SERVO_PWM0 = __BIT(16)
PLATFORM_D_FLAG_DISABLE_DIP_SWITCH = __BIT(17)
PLATFORM_D_FLAG_DISABLE_IMU_HEAD = __BIT(18)
PLATFORM_D_FLAG_DISABLE_IMU_BODY = __BIT(19)
PLATFORM_D_FLAG_DISABLE_I2C_FAIL_WARN = __BIT(20)
PLATFORM_D_FLAG_DISABLE_CLIFF_REFLEX = __BIT(21)
PLATFORM_D_FLAG_DISABLE_KIN_IDLE = __BIT(22)
PLATFORM_D_FLAG_DISABLE_WHEELS = __BIT(23)
PLATFORM_D_FLAG_DISABLE_TRANSLATION = __BIT(24)
PLATFORM_D_FLAG_DISABLE_SPKR_MUTE = __BIT(25)

#	if this is set, the passed flags are stored (else they
#	expire after a timeout)
PLATFORM_D_FLAG_PERSISTENT = __BIT(31)

#	persistent upstream flags (per-session)
PLATFORM_U_FLAG_READY = __BIT(0)
PLATFORM_U_FLAG_PARAMETRISED = __BIT(1)
PLATFORM_U_FLAG_HEAD_JP2 = __BIT(2)
PLATFORM_U_FLAG_HEAD_MOSI_HIGH = __BIT(3)

#	non-persistent upstream flags (per-tick)
PLATFORM_U_FLAG_SPI_FAIL = __BIT(16)
PLATFORM_U_FLAG_I2C_P1H_FAIL = __BIT(17)
PLATFORM_U_FLAG_I2C_P1A_FAIL = __BIT(18)
PLATFORM_U_FLAG_I2C_P1B_FAIL = __BIT(19)
PLATFORM_U_FLAG_WHEELS_STALL = __BIT(20)
PLATFORM_U_FLAG_LOW_BATTERY = __BIT(21)














# warning ignored line: #define INC_MIRO_SOFTWARE_CONSTANTS_H



################################################################
#	WHEELS

#	main drive physical parameters
WHEEL_DIAMETER_M = 0.090
WHEEL_TRACK_M = 0.164
WHEEL_MAX_SPEED_M_PER_S = 0.400

#	parameters derived from the above
WHEEL_CIRCUMF_M = ((3.14159) * WHEEL_DIAMETER_M)
WHEEL_RADIUS_M = (WHEEL_DIAMETER_M * 0.5)
WHEEL_HALF_TRACK_M = (WHEEL_TRACK_M * 0.5)
WHEEL_MAX_ANG_SPEED_RAD_PER_S = (WHEEL_MAX_SPEED_M_PER_S / WHEEL_HALF_TRACK_M)

#	these additional parameters are used to convert between P1
#	signal space (normalised drive downstream and encoder counts
#	upstream) and P2 signal space (m/s both ways). for
#	applications working with the P2 signal space, these parameters
#	are not required, since the conversion is performed in P2.
WHEEL_COUNTS_PER_MOTOR_ROT = 4
WHEEL_GEARBOX_RATIO = 39.822
WHEEL_CMD_SPEED_PER_M_PER_S = 210.0

#	conversions between counts and distance
WHEEL_COUNTS_PER_WHEEL_ROT = (WHEEL_COUNTS_PER_MOTOR_ROT * WHEEL_GEARBOX_RATIO)
WHEEL_M_PER_COUNT = (WHEEL_CIRCUMF_M / WHEEL_COUNTS_PER_WHEEL_ROT)



################################################################
#	PHYSICAL SERVOS

KIN_COUNTS_MIN = 700
KIN_COUNTS_MAX = 2300

COS_COUNTS_MIN = 500
COS_COUNTS_MAX = 2500



################################################################
#	TILT

#	constants associated with TILT DOF (currently fixed).
TILT_RAD_MIN = __DEG2RAD(   -5.0)
TILT_RAD_MAX = TILT_RAD_MIN
TILT_RAD_CALIB = TILT_RAD_MIN



################################################################
#	LIFT

#	constants associated with LIFT DOF.
LIFT_RAD_MIN = __DEG2RAD(   8.0)
LIFT_RAD_MAX = __DEG2RAD(   60.0)
LIFT_RAD_CALIB = __DEG2RAD(   34.0)

LIFT_COUNTS_CALIB = 1600

#	we actually use a nonlinear mapping for this DOF, below,
#	so the value here is not used.
LIFT_COUNTS_PER_RAD = (660 / __DEG2RAD(30.0))






################################################################
#	YAW

#	Constants associated with YAW DOF.
YAW_RAD_MIN = __DEG2RAD(-55.0)
YAW_RAD_MAX = __DEG2RAD(55.0)
YAW_RAD_CALIB = __DEG2RAD(0.0)

YAW_COUNTS_CALIB = 1500
YAW_COUNTS_PER_RAD = (525 / __DEG2RAD(39.0))



################################################################
#	PITCH

#	Constants associated with PITCH DOF.
PITCH_RAD_MIN = __DEG2RAD(  -22.0)
PITCH_RAD_MAX = __DEG2RAD(   8.0)
PITCH_RAD_CALIB = __DEG2RAD(    0.0)

PITCH_COUNTS_CALIB = 1800
PITCH_COUNTS_PER_RAD = (770 / __DEG2RAD(16.8))



################################################################
#	TAIL

#	Constants associated with DROOP DOF.
DROOP_CALIB = 0.0
DROOP_DEFAULT = DROOP_CALIB
DROOP_COUNTS_CALIB = 1500
DROOP_COUNTS_0 = 1500
DROOP_COUNTS_1 = 1370

#	Constants associated with WAG DOF.
WAG_CALIB = 0.5
WAG_DEFAULT = WAG_CALIB
WAG_COUNTS_CALIB = 1500
WAG_COUNTS_0 = 1300
WAG_COUNTS_1 = 1700



################################################################
#	EYELIDS

#	Constants associated with EYELIDS DOFs.
#	"canonical" eye is the left one (for right, invert 0-calib and 1-calib)
EYE_CALIB = 0.5
EYE_DEFAULT = 0.0
EYE_COUNTS_CALIB = 1500
EYE_COUNTS_0 = 1000
EYE_COUNTS_1 = 2000



################################################################
#	EARS

#	Constants associated with EARS DOFs.
#	"canonical" ear is the left one (for right, invert 0-calib and 1-calib)
EAR_CALIB = 0.3333
EAR_DEFAULT = EAR_CALIB
EAR_COUNTS_CALIB = 1500
EAR_COUNTS_0 = 1750
EAR_COUNTS_1 = 1000



################################################################
#	DIMENSIONS

#	location of TILT joint in FOOT
LOC_TILT_X = (0.000)
LOC_TILT_Y = (0.000)
LOC_TILT_Z = (WHEEL_DIAMETER_M * 0.5)

#	location of LIFT joint in BODY
LOC_LIFT_X = (0.0022)
LOC_LIFT_Y = (0.000)
LOC_LIFT_Z = ( (0.0807) - LOC_TILT_Z )

#	location of YAW joint in NECK
LOC_YAW_X = (0.000)
LOC_YAW_Y = (0.000)
LOC_YAW_Z = (0.1201)

#	location of PITCH joint in GIMBAL
LOC_PITCH_X = (0.000)
LOC_PITCH_Y = (0.000)
LOC_PITCH_Z = (0.000)

#	location of nose tip in HEAD
LOC_NOSE_TIP_X = (0.100)
LOC_NOSE_TIP_Y = (0.000)
LOC_NOSE_TIP_Z = (0.025)

#	location of eye (camera optical centre) in HEAD ("canonical" eye is the left one: for right, invert y)
LOC_EYE_X = ( 0.0377)
LOC_EYE_Y = ( 0.0431)
LOC_EYE_Z = ( 0.0568)

#	location of ear (microphone) in HEAD ("canonical" ear is the left one: for right, invert y)
LOC_EAR_X = (-0.0474)
LOC_EAR_Y = ( 0.0519)
LOC_EAR_Z = ( 0.0827)

#	location of tail mic in BODY
LOC_TAIL_MIC_X = (-0.1600)
LOC_TAIL_MIC_Y = ( 0.0000)
LOC_TAIL_MIC_Z = ( (0.0850) - LOC_TILT_Z )

#	location of sonar fovea in HEAD
LOC_SONAR_FOVEA_X = (0.125)
LOC_SONAR_FOVEA_Y = (0.000)
LOC_SONAR_FOVEA_Z = (0.025)

#	region sonar-local fovea can reach
REACHABLE_Z_MIN = 0.070
REACHABLE_Z_MAX = 0.270



################################################################
#	SENSORS

#	camera orientation in head
CAM_ELEVATION = __DEG2RAD( 45.0)
CAM_DIVERGENCE = __DEG2RAD( 27.0)

#	camera projection model (see dev/camera/projection)
CAM_HORI_HALF_FOV = __DEG2RAD(60.0)
CAM_PIXEL_ASPECT_RATIO = 1.0
CAM_DISTORTION_MODEL_K1 = -1.5
CAM_DISTORTION_MODEL_K2 = 1.0
CAM_DISTORTION_MODEL_K3 = -0.5
CAM_DISTORTION_MODEL_K4 = 0.5

#	IMU scaling
ACCEL_1G = (1024)

#	sonar scaling
SONAR_READING_AT_1M = (87.0)



################################################################
#	FLAGS

#	DOCLINK PUSH FLAGS
#	body push types
PUSH_FLAG_IMPULSE = __BIT(0)
PUSH_FLAG_VELOCITY = __BIT(1)
PUSH_FLAG_NO_TRANSLATION = __BIT(2)
PUSH_FLAG_NO_ROTATION = __BIT(3)
PUSH_FLAG_NO_NECK_MOVEMENT = __BIT(4)
PUSH_FLAG_WAIT = __BIT(5)

#	DOCLINK AFFECT FLAGS
#	affect expression flags
AFFECT_EXPRESS_THROUGH_VOICE = __BIT(0)
AFFECT_EXPRESS_THROUGH_NECK = __BIT(1)
AFFECT_EXPRESS_THROUGH_WHEELS = __BIT(2)



################################################################
#	IDENTIFIERS

#	link identifiers
LINK_WORLD = -1
LINK_FOOT = 0
LINK_BODY = 1
LINK_NECK = 2
LINK_GIMBAL = 3
LINK_HEAD = 4
LINK_COUNT = 5

#	kin joint identifiers
JOINT_TILT = 0
JOINT_LIFT = 1
JOINT_YAW = 2
JOINT_PITCH = 3
JOINT_COUNT = 4

#	cos joint identifiers
JOINT_DROOP = 0
JOINT_WAG = 1
JOINT_EYE_L = 2
JOINT_EYE_R = 3
JOINT_EAR_L = 4
JOINT_EAR_R = 5

#	light sensor identifiers
LIGHT_LF = 0
LIGHT_RF = 1
LIGHT_LR = 2
LIGHT_RR = 3

#	cliff sensor identifiers
CLIFF_L = 0
CLIFF_R = 1

#	camera sensor identifiers
CAM_L = 0
CAM_R = 1

#	illum identifiers
ILLUM_LF = 0
ILLUM_LM = 1
ILLUM_LR = 2
ILLUM_RF = 3
ILLUM_RM = 4
ILLUM_RR = 5

#	tone message identifiers
TONE_FREQ = 0
TONE_CODE = 0
TONE_VOL = 1
TONE_DUR = 2



################################################################
#	CONTROL

CLIFF_ADJ_MIN = -8
CLIFF_ADJ_DEFAULT = 0
CLIFF_ADJ_MAX = 8

CLIFF_MEM_MIN = 0
CLIFF_MEM_DEFAULT = 10
CLIFF_MEM_MAX = 50

CLIFF_THRESH_MIN = 0
CLIFF_THRESH_DEFAULT = 6
CLIFF_THRESH_MAX = 15

CLIFF_MARGIN_MIN = 0
CLIFF_MARGIN_DEFAULT = 1
CLIFF_MARGIN_MAX = 4







