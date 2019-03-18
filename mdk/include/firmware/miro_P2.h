
#ifndef INC_MIRO_P2_H
#define INC_MIRO_P2_H

#include "miro_P1.h"



//	upstream group
#define MIRO_MSG_TYPE_P2_U			(MIRO_MSG_TYPE_GROUP_P2 + 0x101)
#define MIRO_MSG_TYPE_FRAME			(MIRO_MSG_TYPE_GROUP_P2 + 0x102)
#define MIRO_MSG_TYPE_MICS			(MIRO_MSG_TYPE_GROUP_P2 + 0x103)

//	downstream group
#define MIRO_MSG_TYPE_P2_D			(MIRO_MSG_TYPE_GROUP_P2 + 0x201)
#define MIRO_MSG_TYPE_STREAM		(MIRO_MSG_TYPE_GROUP_P2 + 0x202)

//	config group
#define MIRO_MSG_TYPE_PLATFORM_PARS	(MIRO_MSG_TYPE_GROUP_P2 + 0x301)
#define MIRO_MSG_TYPE_CONFIG		(MIRO_MSG_TYPE_GROUP_P2 + 0x302)
#define MIRO_MSG_TYPE_CONFIG_REQ	(MIRO_MSG_TYPE_GROUP_P2 + 0x303)
#define MIRO_MSG_TYPE_BLANK_P2PROG	(MIRO_MSG_TYPE_GROUP_P2 + 0x304)
#define MIRO_MSG_TYPE_RESET			(MIRO_MSG_TYPE_GROUP_P2 + 0x305)
#define MIRO_MSG_TYPE_RESET_USB		(MIRO_MSG_TYPE_GROUP_P2 + 0x306)

//	flash group
#define MIRO_MSG_TYPE_FLASH			(MIRO_MSG_TYPE_GROUP_P2 + 0x401)
#define MIRO_MSG_TYPE_FLASH_RESULT	(MIRO_MSG_TYPE_GROUP_P2 + 0x402)



////////////////////////////////////////////////////////////////
//
//	MIRO_P2_CONFIG
//
////////////////////////////////////////////////////////////////

/*
	The CONFIG object is stored in FLASH above the program and
	can be used to configure the behaviour of the platform
	before it is accessed by the bridge, also to store any
	identification information for the robot.

	Currently, robots do not have a stored CONFIG object, and
	just load the default at start-up.
*/

#define MIRO_P2_CONFIG_ADDR							0x080E0000

#define MIRO_P2_CONFIG_SIGNATURE					0x12345678
#define MIRO_P2_CONFIG_VERSION						1
#define MIRO_P2_CONFIG_SERIAL_NUMBER_LENGTH			32

#define MIRO_P2_CONFIG_FLAG_IGNORE_HIGH_MOSI		__BIT(0)

struct MIRO_P2_CONFIG
{
	//	header
	uint32_t signature;
	uint32_t version;
	uint32_t checksum;

	//	flags can be set by user if required
	uint32_t flags;

	//	serial number may be set at factory or by user; for now,
	//	we do not actually use this storage in P2, but it is
	//	reserved in case we want to use it in future.
	char serial_number__reserved[MIRO_P2_CONFIG_SERIAL_NUMBER_LENGTH];

	//	identity information is provided by device manufacturer,
	//	and recovered at run-time. it is included here only for
	//	convenience of returning it to the caller
	uint32_t ident[4];

	//	reserved space
	uint32_t __reserved[240];
};

#define MIRO_P2_CONFIG_SIZE 1024
__PREPROC_ASSERT_SIZE(MIRO_P2_CONFIG);

#define __MIRO_P2_CONFIG_DEFAULT(config) do { \
	uint32_t* p = (uint32_t*) &config; \
	for (uint32_t i=0; i<(sizeof(config)/sizeof(uint32_t)); i++) \
		p[i] = 0; \
	config.signature = MIRO_P2_CONFIG_SIGNATURE; \
	config.version = MIRO_P2_CONFIG_VERSION; \
} while(false)



////////////////////////////////////////////////////////////////
//
//	MIRO_P2_PARS
//
////////////////////////////////////////////////////////////////

struct MIRO_FRAME_GEOM
{
	uint16_t sx;	//	number of columns (rows) in the frame
	uint16_t sy;
	uint16_t ox;	//	offset at left (top) edge (use 0 for full frame)
	uint16_t oy;
	uint16_t kx;	//	number of columns (rows) skipped between each column (use 0 for full frame)
	uint16_t ky;

	//	align
	uint16_t __align[2];
};

struct MIRO_FRAME_SIZE
{
	uint16_t sx;	//	number of columns (rows) in the frame
	uint16_t sy;
};

struct MIRO_VIDEO_PARS
{
	float frame_rate;		//	use 0.0 to disable this camera
	uint8_t jpeg_quality;	//	0 to 255, low to high

	//	align
	uint8_t __align[3];
};

struct MIRO_CAMERA_CONFIG
{
	struct MIRO_FRAME_GEOM frame_geom;
	struct MIRO_VIDEO_PARS video_pars;
};

//	currently, only the listed values are supported - expansion is planned for the future
#define MIRO_FRAME_GEOM_720w ((struct MIRO_FRAME_GEOM) { 1280, 720, 0, 0, 0, 0, {0, 0} })
#define MIRO_FRAME_GEOM_720s ((struct MIRO_FRAME_GEOM) { 960,  720, 0, 0, 0, 0, {0, 0} })
#define MIRO_FRAME_GEOM_360w ((struct MIRO_FRAME_GEOM) { 640,  360, 0, 0, 0, 0, {0, 0} })
#define MIRO_FRAME_GEOM_360s ((struct MIRO_FRAME_GEOM) { 480,  360, 0, 0, 0, 0, {0, 0} })
#define MIRO_FRAME_GEOM_240s ((struct MIRO_FRAME_GEOM) { 320,  240, 0, 0, 0, 0, {0, 0} })
#define MIRO_FRAME_GEOM_180w ((struct MIRO_FRAME_GEOM) { 320,  180, 0, 0, 0, 0, {0, 0} })
#define MIRO_FRAME_GEOM_180s ((struct MIRO_FRAME_GEOM) { 240,  180, 0, 0, 0, 0, {0, 0} })

//	any frame rate can be attempted, but only some will be actioned by the firmware
#define MIRO_VIDEO_PARS_OFF ((struct MIRO_VIDEO_PARS) { 0.0f, 192, {0, 0, 0} })
#define MIRO_VIDEO_PARS_10_HIQ ((struct MIRO_VIDEO_PARS) { 10.0f, 192, {0, 0, 0} })
#define MIRO_VIDEO_PARS_15_HIQ ((struct MIRO_VIDEO_PARS) { 15.0f, 192, {0, 0, 0} })

//	P2 parameters version
#define MIRO_P2_PARS_VERSION					1

//	servo sensor map
struct MIRO_SERVO_SENSOR_MAP
{
	int32_t m;
	int32_t c;
};

//	packing
#define MIRO_P2_PARS_PACKING_COUNT 256

struct MIRO_P2_PARS
{
	//	header
	uint16_t flags; // see MIRO_P1_PARS
	uint8_t version;
	uint8_t __align;

	//	camera
	struct
	{
		struct MIRO_CAMERA_CONFIG config[2];
	}
	camera;

	//	servos
	struct
	{
		//	sensor mappings
		struct MIRO_SERVO_SENSOR_MAP lift;
		struct MIRO_SERVO_SENSOR_MAP yaw;
		struct MIRO_SERVO_SENSOR_MAP pitch;
	}
	servos;

	//	battery
	struct
	{
		uint16_t low_bat_warning_mv;
		uint16_t low_bat_warning_period;
	}
	battery;

	//	expansion
	uint8_t __packing[MIRO_P2_PARS_PACKING_COUNT];
};

//	dataset 190107-REF (Reference servos received 190107)
//	note now similar gearing on yaw/pitch, lift is unchanged
#define MIRO_SERVO_SENSOR_MAP_LIFT_190107 \
	((struct MIRO_SERVO_SENSOR_MAP){-71610, 3124})
#define MIRO_SERVO_SENSOR_MAP_YAW_190107 \
	((struct MIRO_SERVO_SENSOR_MAP){-86344, 3615})
#define MIRO_SERVO_SENSOR_MAP_PITCH_190107 \
	((struct MIRO_SERVO_SENSOR_MAP){-87054, 3494})

#define __MIRO_P2_PARS_DEFAULT(x) do { \
	x.flags = 0; \
	x.version = MIRO_P2_PARS_VERSION; \
	x.__align = 0; \
	for (int c=0; c<2; c++) { \
		x.camera.config[c].frame_geom = MIRO_FRAME_GEOM_360w; \
		x.camera.config[c].video_pars = MIRO_VIDEO_PARS_15_HIQ; \
	} \
	x.servos.lift = MIRO_SERVO_SENSOR_MAP_LIFT_190107; \
	x.servos.yaw = MIRO_SERVO_SENSOR_MAP_YAW_190107; \
	x.servos.pitch = MIRO_SERVO_SENSOR_MAP_PITCH_190107; \
	x.battery.low_bat_warning_mv = 4600; \
	x.battery.low_bat_warning_period = 500; \
	for (int i=0; i<MIRO_P2_PARS_PACKING_COUNT; i++) x.__packing[i] = 0; \
} while(false)

//	DOCLINK cameras off by default
//	NB: the default camera parameters are sent down by the bridge; the
//	firmware keeps the cameras off until the bridge connects (DOCLINK).
#define __MIRO_P2_PARS_CAMERAS_OFF(x) do { \
	for (int c=0; c<2; c++) { \
		x.camera.config[c].video_pars = MIRO_VIDEO_PARS_OFF; \
	} \
} while(false)

//	design size
#define MIRO_PLATFORM_PARS_SIZE 2048

struct MIRO_PLATFORM_PARS
{
	//	header
	uint16_t flags; // see MIRO_P1_PARS
	uint16_t checksum; // used to confirm a parameter set stored in P2 FLASH

	//	parameter blocks for each processor set
	struct MIRO_P1_PARS P1;
	struct MIRO_P2_PARS P2;

	//	packing
	uint8_t __packing[MIRO_PLATFORM_PARS_SIZE-sizeof(struct MIRO_P1_PARS)-sizeof(struct MIRO_P2_PARS)-4];
};

__PREPROC_ASSERT_SIZE(MIRO_PLATFORM_PARS)

#define __MIRO_PLATFORM_PARS_DEFAULT(x) do { \
	__MIRO_P1_PARS_DEFAULT(x.P1); \
	__MIRO_P2_PARS_DEFAULT(x.P2); \
} while(false)

struct MIRO_MSG_PLATFORM_PARS
{
	struct MIRO_MANIFEST manifest;
	struct MIRO_PLATFORM_PARS pars;
};



////////////////////////////////////////////////////////////////
//
//	MIRO_P2_U/D
//
////////////////////////////////////////////////////////////////

struct MIRO_P2_U
{
	struct MIRO_MANIFEST manifest;

	//	platform
	struct MIRO_PLATFORM_U platform_u;
	struct MIRO_P1P_U P1P;

	//	P2
	int32_t t_now;
	uint16_t stream_buffer_space;
	uint16_t stream_buffer_total;
	uint16_t P1A_release;
	uint16_t P1B_release;
	uint16_t P1H_release;
	uint16_t __align;
	uint32_t P2_checksum;
	uint32_t P1A_checksum;
	uint32_t P1B_checksum;
	uint32_t P1H_checksum;
	struct MIRO_USR_SPI_U usr_spi;

	//	P1a
	struct MIRO_WHEELS_U wheels;
	struct MIRO_DIP_U dip;
	struct MIRO_LIGHT_U light;
	struct MIRO_BATTERY_U battery;
	struct MIRO_TEMPERATURE_U temperature;
	struct MIRO_IMU_U imu_body;
	struct MIRO_TOUCH_U touch_body;

	//	P1b
	struct MIRO_SERVO_U yaw;
	struct MIRO_SERVO_U lift;
	struct MIRO_CLIFF_U cliff;
	struct MIRO_MIC_U mic;
	struct MIRO_USR_I2C_U usr_i2c;

	//	P1h
	struct MIRO_SERVO_U pitch;
	struct MIRO_SONAR_U sonar;
	struct MIRO_IMU_U imu_head;
	struct MIRO_TOUCH_U touch_head;
};

struct MIRO_P2_D
{
	struct MIRO_MANIFEST manifest;

	//	platform
	struct MIRO_PLATFORM_D platform_d;
	struct MIRO_P1P_D P1P;

	//	cosmetic
	struct MIRO_TONE_D tone;
	struct MIRO_ILLUM_D illum[2];
	struct MIRO_SERVO_D droop;
	struct MIRO_SERVO_D wag;
	struct MIRO_SERVO_D ear[2];
	struct MIRO_SERVO_D eye[2];

	//	kinematic
	struct MIRO_WHEELS_D wheels;
	struct MIRO_SERVO_D lift;
	struct MIRO_SERVO_D yaw;
	struct MIRO_SERVO_D pitch;

	//	other
	struct MIRO_USR_I2C_D usr_i2c;
	struct MIRO_USR_SPI_D usr_spi;
};



////////////////////////////////////////////////////////////////
//
//	MIRO_MSG_CONFIG
//
////////////////////////////////////////////////////////////////

//	this message is sent up with a copy of the stored config
//	object, only on request
struct MIRO_MSG_CONFIG
{
	struct MIRO_MANIFEST manifest;
	struct MIRO_P2_CONFIG config;
};



////////////////////////////////////////////////////////////////
//
//	MIRO_MSG_FRAME
//
////////////////////////////////////////////////////////////////

//	this message contains a video frame, or part thereof; if it
//	is only a part, block_index will indicate which part, the
//	flag LAST_BLOCK will indicate that it is the last part, and
//	the complete frame can be recovered simply by concatenating
//	the data segments of all parts, in block_index order.

//	constants
#define MIRO_STREAM_INDEX_CAML				0
#define MIRO_STREAM_INDEX_CAMR				1
#define MIRO_STREAM_INDEX_GOPRO				2
#define MIRO_STREAM_COUNT					3

//	flags
#define MIRO_FRAME_FLAG_LAST_BLOCK			__BIT(0)
#define MIRO_FRAME_FLAG_RGB_DATA			__BIT(1)
#define MIRO_FRAME_FLAG_JPEG_DATA			__BIT(2)

struct MIRO_FRAME_HEADER
{
	//	frame identification data
	uint16_t frame_index;		//	index of frame as delivered at cameras
	uint8_t stream_index;		//	any MIRO_STREAM_INDEX_* constant
	uint8_t block_index;		//	index of this frame block in frame

	//	frame flags
	uint16_t flags;

	//	spare
	uint16_t __spare[3];

	//	frame spec
	struct MIRO_FRAME_SIZE size;

	//	timing
	int32_t t_start;			//	time recording of this frame began
	int32_t t_complete;			//	time recording of this frame completed

	//	this header is now followed by any number of bytes which represent
	//	a block of data from the JPEG data stream to be reassembled at the
	//	destination into the complete JPEG frame
};

#define MIRO_FRAME_HEADER_SIZE 24
__PREPROC_ASSERT_SIZE(MIRO_FRAME_HEADER);

struct MIRO_MSG_FRAME
{
	struct MIRO_MANIFEST manifest;
	struct MIRO_FRAME_HEADER header;
	uint8_t data[]; // actual data size defined by manifest->msg_size
};



////////////////////////////////////////////////////////////////
//
//	MIRO_MSG_MICS
//
////////////////////////////////////////////////////////////////

/*
	A MIRO_MSG_MICS message comprises a manifest, sync data, and a
	block of int16_t numbers representing the sampled data. We
	pack the data in MIRO_MIC_SAMPLES_PER_BLOCK that is /40 times fS
	rate, because that leaves some leeway versus the /50 that would
	be required in principle so we don't drop data.
*/

#define MIRO_MIC_NUM_CHAN_HEAD			3
#define MIRO_MIC_NUM_WORDS_PER_MSG		(MIRO_MIC_NUM_CHAN_HEAD * MIRO_MIC_SAMPLES_PER_BLOCK)

struct MIRO_MSG_MICS
{
	struct MIRO_MANIFEST manifest;
	struct MIRO_MIC_HEADER header;
	int16_t data[MIRO_MIC_NUM_WORDS_PER_MSG];
};

#define MIRO_MSG_MICS_SIZE (sizeof(struct MIRO_MSG_MICS))



////////////////////////////////////////////////////////////////
//
//	MIRO_MSG_FLASH
//
////////////////////////////////////////////////////////////////

#define MIRO_FLASH_BLOCK_SIZE			256

#define MIRO_FLASH_FLAG_FIRST_BLOCK		(1 << 0)
#define MIRO_FLASH_FLAG_LAST_BLOCK		(1 << 1)
#define MIRO_FLASH_FLAG_SLOWLY			(1 << 2)

enum MIRO_FLASH_TARGET
{
	MIRO_FLASH_TARGET_NONE = 0,
	MIRO_FLASH_TARGET_P1_ALL
};

struct MIRO_MSG_FLASH
{
	struct MIRO_MANIFEST manifest;
	uint16_t flags;
	uint16_t __align;
	enum MIRO_FLASH_TARGET target;
	uint32_t addr0; // target block start
	uint32_t addr1; // target block end
	uint8_t data[MIRO_FLASH_BLOCK_SIZE];
};

#define MIRO_MSG_FLASH_SIZE (MIRO_MANIFEST_SIZE + 16 + MIRO_FLASH_BLOCK_SIZE)
__PREPROC_ASSERT_SIZE(MIRO_MSG_FLASH)

struct MIRO_MSG_FLASH_RESULT
{
	struct MIRO_MANIFEST manifest;
	uint32_t flags;
	uint32_t addr0;
	uint32_t addr1;
	uint32_t __align;
};

#define MIRO_MSG_FLASH_RESULT_SIZE (MIRO_MANIFEST_SIZE + 16)
__PREPROC_ASSERT_SIZE(MIRO_MSG_FLASH_RESULT)



////////////////////////////////////////////////////////////////
//
//	MIRO_MSG_STREAM
//
////////////////////////////////////////////////////////////////

struct MIRO_MSG_STREAM
{
	struct MIRO_MANIFEST manifest;
	uint8_t data[]; // actual data size defined by manifest->msg_size
};



#endif



