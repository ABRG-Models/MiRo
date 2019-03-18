
#ifndef INC_MIRO_P0_H
#define INC_MIRO_P0_H



#include "miro_msg.h"



////////////////////////////////////////////////////////////////
//
//	PLATFORM
//
////////////////////////////////////////////////////////////////

//	most (all?) flags are defined in miro_constants.h

#define MIRO_PLATFORM_D_SIGNAL_CONNECTED			__BIT(0)
#define MIRO_PLATFORM_D_SIGNAL_HEARTBEAT			__BIT(1)
#define MIRO_PLATFORM_D_SIGNAL_TEST_OK				__BIT(2)
#define MIRO_PLATFORM_D_SIGNAL_TEST_HEAD			__BIT(3)
#define MIRO_PLATFORM_D_SIGNAL_TEST_LEFT			__BIT(4)

#define MIRO_PLATFORM_D_SIGNAL_DEBUG_0				__BIT(8)
#define MIRO_PLATFORM_D_SIGNAL_DEBUG_1				__BIT(9)
#define MIRO_PLATFORM_D_SIGNAL_DEBUG_2				__BIT(10)
#define MIRO_PLATFORM_D_SIGNAL_DEBUG_3				__BIT(11)
#define MIRO_PLATFORM_D_SIGNAL_DEBUG_4				__BIT(12)
#define MIRO_PLATFORM_D_SIGNAL_DEBUG_5				__BIT(13)
#define MIRO_PLATFORM_D_SIGNAL_DEBUG_6				__BIT(14)
#define MIRO_PLATFORM_D_SIGNAL_DEBUG_7				__BIT(15)

#define MIRO_PLATFORM_D_COMMAND_SHOW_SERVO_SENSORS	1

struct MIRO_PLATFORM_D
{
	uint32_t tick;
	uint16_t command;
	uint16_t signals;
	uint16_t __align1[2];
	uint32_t flags;
	uint16_t __align2[8];
};

//	note that platform_u.flags are set by various sources (P1a,
//	P1b, P1h, P2, P3 even) but are all listed here so that they
//	can be OR'd together for delivery over the ROS interface

//	persistent flags must be in this range
#define MIRO_PLATFORM_U_FLAG_PERSISTENT_MASK		0xFFFF

//	most flags are defined in miro_constants.h

//	flags used during test (not during production)
#define MIRO_PLATFORM_U_FLAG_TEST_COMPLETE			__BIT(31)

//	these flags are loaded into "response" only during test
#define MIRO_PLATFORM_U_TEST_FAIL_BODY_TOUCH		__BIT(0)
#define MIRO_PLATFORM_U_TEST_FAIL_ILLUM0			__BIT(1)
#define MIRO_PLATFORM_U_TEST_FAIL_ILLUM1			__BIT(2)
#define MIRO_PLATFORM_U_TEST_FAIL_COMM_P1AB			__BIT(3)
#define MIRO_PLATFORM_U_TEST_FAIL_BUCK_P1B			__BIT(4)
#define MIRO_PLATFORM_U_TEST_FAIL_COMM_P1BA			__BIT(5)
#define MIRO_PLATFORM_U_TEST_FAIL_COMM_P1AP			__BIT(6)
#define MIRO_PLATFORM_U_TEST_FAIL_BUCK_P1P			__BIT(7)
#define MIRO_PLATFORM_U_TEST_FAIL_R78				__BIT(8)
#define MIRO_PLATFORM_U_TEST_FAIL_COMM_P1PB			__BIT(9)
#define MIRO_PLATFORM_U_TEST_FAIL_COMM_P1PA_CS		__BIT(10)
#define MIRO_PLATFORM_U_TEST_FAIL_COMM_P1BA_CS		__BIT(11)
#define MIRO_PLATFORM_U_TEST_FAIL_COMM_P1PA			__BIT(12)
#define MIRO_PLATFORM_U_TEST_FAIL_HEAD_TOUCH		__BIT(13)
#define MIRO_PLATFORM_U_TEST_FAIL_SERVOS			__BIT(14)
#define MIRO_PLATFORM_U_TEST_FAIL_UNEXPECTED		__BIT(15)

struct MIRO_PLATFORM_U
{
	uint32_t tick;
	uint16_t response;
	uint16_t __align1[1];
	uint32_t checksum;
	uint32_t flags;
	uint16_t __align2[8];
};

#define MIRO_LOG_N 32

struct MIRO_LOG_U
{
	uint8_t data[MIRO_LOG_N]; // always null-terminated
};

enum LED_MODE
{
	LED_NORMAL = 0, // indicate power state in the usual way
	LED_NORMAL_X, // NORMAL but OFF when in most common power state, POWER_ON
	LED_HEARTBEAT, // follow system heartbeat
	LED_OFF, // always off
	LED_ON // always on
};

struct MIRO_P1P_D
{
	uint8_t power_led_mode; // each nibble is the LED_MODE of one of the power state leds
	uint8_t __align[3];
};

struct MIRO_P1P_U
{
	uint16_t release;
	uint16_t __align[1];
	uint32_t checksum;
};



////////////////////////////////////////////////////////////////
//
//	SERVO
//
////////////////////////////////////////////////////////////////

//	this flag must be set or the servo will de-energise (this is used
//	to determine that a command signal was sent at all)
#define MIRO_SERVO_FLAG_ENABLE __BIT(0)

//	servo controller will usually auto power off after some fixed idle
//	period; setting this flag forces the servo to remain energised.
#define MIRO_SERVO_FLAG_NO_IDLE __BIT(1)

//	if TEST is specified, the local controller will range the servo
//	back and forth continuously.
#define MIRO_SERVO_FLAG_TEST __BIT(2)

//	indicates that the sensor data in "meas" is valid
#define MIRO_SERVO_FLAG_MEAS __BIT(3)

//	indicates that the servo's sensor should be used
#define MIRO_SERVO_FLAG_USE_SENSOR __BIT(4)

struct MIRO_SERVO_D
{
	uint16_t cmd_usec;		//	target position in microseconds (some positions not achievable)
	uint16_t meas_usec;		//	sensed position fed back from sensor wire (converted to usec)
	uint8_t speed;			//	slew rate: 0 is "stationary"; 255 is "unlimited" (effectively)
	uint8_t flags;
	uint8_t __align[2];
};

struct MIRO_SERVO_U
{
    uint16_t raw_ain;		//	used in auto-calibration of servo sensor pot
    uint16_t meas_usec;		//	position as measured by potentiometer sensor (raw AIN to P2, then converted to usec)
    uint8_t flags;
    uint8_t __align[3];
};



////////////////////////////////////////////////////////////////
//
//	WHEELS
//
////////////////////////////////////////////////////////////////

//	this power-2 exponent defines the scale of signals upstream and
//	downstream to the wheel controller (maximum speed and effort are
//	both around 1.0, at this scaling).
#define MIRO_WHEELS_SIGNAL_EXPONENT 12
#define MIRO_WHEELS_SIGNAL_SCALE (1 << MIRO_WHEELS_SIGNAL_EXPONENT)

struct MIRO_WHEELS_D
{
	int16_t speed_cmd[2];
};

struct MIRO_WHEELS_U
{
	int16_t speed_opto[2];
	int16_t speed_back_emf[2];
	int16_t effort_pwm[2];
};

struct MIRO_WHEELS_U2
{
	int16_t speed_opto[2];
};



////////////////////////////////////////////////////////////////
//
//	TEMPERATURE
//
////////////////////////////////////////////////////////////////

struct MIRO_TEMPERATURE_U
{
	float temperature;
};



////////////////////////////////////////////////////////////////
//
//	BATTERY
//
////////////////////////////////////////////////////////////////

struct MIRO_BATTERY_U
{
	float voltage;
};



////////////////////////////////////////////////////////////////
//
//	AUDIO
//
////////////////////////////////////////////////////////////////

#define MIRO_STREAM_SAMPLES_PER_TICK	256

struct MIRO_SPKR_U
{
	uint16_t stream_buffer_samples_remaining;
	uint16_t stream_buffer_samples_total;
};

struct MIRO_TONE_D
{
	union
	{
		uint16_t freq;
		uint16_t code;
	};
	uint8_t volume;
	uint8_t duration;
};

#define MIRO_SPKR_FLAG_SILENT			__BIT(0)

struct MIRO_SPKR_D
{
	struct MIRO_TONE_D tone;
	uint8_t stream_samples[MIRO_STREAM_SAMPLES_PER_TICK];
	uint16_t stream_sample_count;
	uint8_t flags;
	uint8_t __align[1];
};



////////////////////////////////////////////////////////////////
//
//	MIC
//
////////////////////////////////////////////////////////////////

#define MIRO_MIC_SAMPLES_PER_BLOCK		500
#define MIRO_MIC_SYNC_SAMPLE_NO_SYNC	0x7FFF
#define MIRO_MIC_SYNC_SAMPLE_NO_DATA	0xFFFF

struct MIRO_MIC_HEADER
{
	uint16_t sync_id;
	uint16_t sync_sample;
};

struct MIRO_MIC_U
{
	struct MIRO_MIC_HEADER header;
	int16_t data[MIRO_MIC_SAMPLES_PER_BLOCK];
};

struct MIRO_MIC_D
{
	uint16_t sync_id;
	uint8_t __align[2];
};



////////////////////////////////////////////////////////////////
//
//	DIP
//
////////////////////////////////////////////////////////////////

struct MIRO_DIP_U
{
    uint32_t state;
};



////////////////////////////////////////////////////////////////
//
//	LIGHT
//
////////////////////////////////////////////////////////////////

#define MIRO_LIGHT_COUNT 4

struct MIRO_LIGHT_U
{
    uint16_t sensor[MIRO_LIGHT_COUNT];
};



////////////////////////////////////////////////////////////////
//
//	CLIFF
//
////////////////////////////////////////////////////////////////

#define MIRO_CLIFF_COUNT 2

struct MIRO_CLIFF_U
{
    uint8_t sensor[MIRO_CLIFF_COUNT];
    uint8_t __align[2];
};



////////////////////////////////////////////////////////////////
//
//	ILLUM
//
////////////////////////////////////////////////////////////////

#define MIRO_ILLUM_COUNT 3

struct MIRO_ILLUM_D
{
	uint32_t argb[MIRO_ILLUM_COUNT];
};



////////////////////////////////////////////////////////////////
//
//	USR_I2C
//
////////////////////////////////////////////////////////////////

#define MIRO_USR_I2C_NUM_OPS 8
#define MIRO_USR_I2C_BLOCK_BYTES 32
#define MIRO_USR_I2C_OP_RESULT_NOP 0
#define MIRO_USR_I2C_OP_RESULT_OK 1
#define MIRO_USR_I2C_OP_RESULT_FAIL 2
#define MIRO_USR_I2C_OP_RESULT_BAD 3

struct MIRO_I2C_OP
{
	uint8_t addr;			//	LSb is READ bit (0=W, 1=R)
	uint8_t duty_length;	//	LS 4 bits is (length-1) (so, up to 16 bytes); MS 4 bits are cycle bits (0b1111 = every cycle)
	uint8_t device_reg;		//	register of device to access
	uint8_t block_reg;		//	register of data block (read or write) to access
};

struct MIRO_USR_I2C_D
{
	struct MIRO_I2C_OP op[MIRO_USR_I2C_NUM_OPS];
	uint8_t block[MIRO_USR_I2C_BLOCK_BYTES];
};

struct MIRO_USR_I2C_U
{
	uint8_t op_result[MIRO_USR_I2C_NUM_OPS];
	uint8_t block[MIRO_USR_I2C_BLOCK_BYTES];
};



////////////////////////////////////////////////////////////////
//
//	USR_SPI
//
////////////////////////////////////////////////////////////////

#define MIRO_USR_SPI_MAX_WORDS 64

#define MIRO_USR_SPI_CLOCK_PERIOD_NSEC 320
#define MIRO_USR_SPI_WORD_PERIOD_NSEC (MIRO_USR_SPI_CLOCK_PERIOD_NSEC * 16)
#define MIRO_USR_SPI_EXCHANGE_PERIOD_USEC (MIRO_USR_SPI_WORD_PERIOD_NSEC * MIRO_USR_SPI_MAX_WORDS / 1000)
#define MIRO_USR_SPI_WAIT_USEC 6
#define MIRO_USR_SPI_CS_MARGIN_USEC 2
#define MIRO_USR_SPI_LEAD_IN_USEC (MIRO_USR_SPI_WAIT_USEC + MIRO_USR_SPI_CS_MARGIN_USEC)
#define MIRO_USR_SPI_LEAD_OUT_USEC (MIRO_USR_SPI_CS_MARGIN_USEC)
#define MIRO_USR_SPI_BUSY_PERIOD_USEC (MIRO_USR_SPI_LEAD_IN_USEC + MIRO_USR_SPI_EXCHANGE_PERIOD_USEC + MIRO_USR_SPI_LEAD_OUT_USEC)

struct MIRO_USR_SPI_D
{
	uint16_t count;
	uint16_t __align;
	uint16_t data[MIRO_USR_SPI_MAX_WORDS];
};

struct MIRO_USR_SPI_U
{
	uint16_t data[MIRO_USR_SPI_MAX_WORDS];
};



////////////////////////////////////////////////////////////////
//
//	IMU
//
////////////////////////////////////////////////////////////////

#define MIRO_IMU_1G 8192

struct MIRO_IMU_U
{
	struct
	{
		union
		{
			int16_t chan[3];

			struct
			{
				int16_t x;
				int16_t y;
				int16_t z;
			};
		}
		accel;

		uint16_t temperature;

		union
		{
			int16_t chan[3];

			struct
			{
				int16_t x;
				int16_t y;
				int16_t z;
			};
		}
		gyro;

		union
		{
			int16_t chan[3];

			struct
			{
				int16_t x;
				int16_t y;
				int16_t z;
			};
		}
		mag;
	};
};



////////////////////////////////////////////////////////////////
//
//	TOUCH
//
////////////////////////////////////////////////////////////////

struct MIRO_TOUCH_U
{
	uint32_t sensor;
};



////////////////////////////////////////////////////////////////
//
//	SONAR
//
////////////////////////////////////////////////////////////////

#define MIRO_SONAR_ECHO_COUNT 4

struct MIRO_SONAR_U
{
	int16_t echo_usec[MIRO_SONAR_ECHO_COUNT];
};



#endif



