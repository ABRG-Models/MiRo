
#ifndef INC_MIRO_P1_H
#define INC_MIRO_P1_H

#include "miro_P0.h"



////////////////////////////////////////////////////////////////
//
//	PARS
//
////////////////////////////////////////////////////////////////

//	the P1 pars object is common to all the P1 devices.

//	this power-2 exponent defines the scale of gains in the
//	wheel controller (unity gain being 1.0).
#define MIRO_WHEELS_GAIN_EXPONENT 12

//	this flag is used by P1, P2, and PLATFORM_PARS
#define MIRO_PARS_FLAG_UPDATED					__BIT(0)

//	P1 parameters version
#define MIRO_P1_PARS_VERSION					1

//	ADC is 12-bit, referenced to 3.3V, and voltage divider scales to 40%
#define MIRO_BATTERY_VOLTAGE_SCALE ((3.3f / 4095.0f) * 2.5f)

//	packing
#define MIRO_P1_PARS_PACKING_COUNT 24

struct MIRO_P1_PARS
{
	//	header
	uint16_t flags;
	uint8_t version;
	uint8_t __align;

	//	comms
	struct
	{
		uint16_t SPI_USR_usec;
		uint16_t __align;
	}
	comms;

	//	wheels
	struct
	{
		//	speed controller gains
		uint16_t gain_S0;
		uint16_t gain_S1;
		uint16_t gain_S2;
		uint16_t gain_S3;
		uint16_t gain_P;
		uint16_t gain_I;
		uint16_t gain_D;

		//	other controller parameters
		uint16_t samples_before_depower;
		uint8_t input_filter_bits;
		uint8_t __align[3];

		//	integral limit
		int32_t integral_limit;

		//	vbat limits
		uint16_t vbat_lo;
		uint16_t vbat_hi;
	}
	wheels;

	//	touch
	struct
	{
		uint8_t sensitivity_body; // 0 (low) to 15 (high)
		uint8_t sensitivity_head;
		uint16_t __align;
	}
	touch;

	//	expansion
	uint8_t __packing[MIRO_P1_PARS_PACKING_COUNT];
};

//	confirm size for multiple compilers
#define MIRO_P1_PARS_SIZE (64)
__PREPROC_ASSERT_SIZE_IS(MIRO_P1_PARS, MIRO_P1_PARS_SIZE)

#define MIRO_WHEELS_GAIN_SCALE (1 << MIRO_WHEELS_GAIN_EXPONENT)

#define __MIRO_P1_PARS_DEFAULT(x) do { \
	x.flags = 0; \
	x.version = MIRO_P1_PARS_VERSION; \
	x.__align = 0; \
	x.comms.SPI_USR_usec = 100; \
	x.comms.__align = 0; \
	x.wheels.gain_S0 = 0.2 * MIRO_WHEELS_GAIN_SCALE; \
	x.wheels.gain_S1 = 0.3 * MIRO_WHEELS_GAIN_SCALE; \
	x.wheels.gain_S2 = 0.1 * MIRO_WHEELS_GAIN_SCALE; \
	x.wheels.gain_S3 = 0.0 * MIRO_WHEELS_GAIN_SCALE; \
	x.wheels.gain_P =  0.5 * MIRO_WHEELS_GAIN_SCALE; \
	x.wheels.gain_I =  0.0 * MIRO_WHEELS_GAIN_SCALE; \
	x.wheels.gain_D =  2.0 * MIRO_WHEELS_GAIN_SCALE; \
	x.wheels.integral_limit = 8.000 * MIRO_WHEELS_GAIN_SCALE; \
	x.wheels.samples_before_depower = 10; \
	x.wheels.input_filter_bits = 3; \
	x.wheels.vbat_lo = 4.0f / MIRO_BATTERY_VOLTAGE_SCALE; \
	x.wheels.vbat_hi = 4.2f / MIRO_BATTERY_VOLTAGE_SCALE; \
	x.touch.sensitivity_body = 8; \
	x.touch.sensitivity_head = 8; \
	x.touch.__align = 0; \
	for (int i=0; i<MIRO_P1_PARS_PACKING_COUNT; i++) x.__packing[i] = 0; \
} while(false)



////////////////////////////////////////////////////////////////
//
//	MIRO_P1A
//
////////////////////////////////////////////////////////////////

struct MIRO_P1A_U
{
	struct MIRO_PLATFORM_U platform_u; // 32
	struct MIRO_P1P_U P1P; // 8
	struct MIRO_LOG_U log; // 32
	struct MIRO_WHEELS_U wheels; // 12
	struct MIRO_DIP_U dip; // 4
	struct MIRO_LIGHT_U light; // 8
	struct MIRO_BATTERY_U battery; // 4
	struct MIRO_TEMPERATURE_U temperature; // 4
	struct MIRO_IMU_U imu; // 20
	struct MIRO_TOUCH_U touch; // 4
	struct MIRO_SERVO_U yaw; // 8
	struct MIRO_SERVO_U lift; // 8
};

struct MIRO_P1A_D
{
	struct MIRO_P1_PARS pars; // 64

	struct MIRO_PLATFORM_D platform_d; // 32
	struct MIRO_P1P_D P1P; // 4
	struct MIRO_WHEELS_D wheels; // 4
	struct MIRO_ILLUM_D illum[2]; // 24 (12 x 2)

	//	make structures same size
	uint8_t __packing[16];
};

//	confirm size for multiple compilers
#define MIRO_P1A_SIZE (144)
__PREPROC_ASSERT_SIZE_IS(MIRO_P1A_U, MIRO_P1A_SIZE)
__PREPROC_ASSERT_SIZE_IS(MIRO_P1A_D, MIRO_P1A_SIZE)



////////////////////////////////////////////////////////////////
//
//	MIRO_P1B
//
////////////////////////////////////////////////////////////////

struct MIRO_P1B_U
{
	struct MIRO_PLATFORM_U platform_u; // 32
	struct MIRO_LOG_U log; // 32
	struct MIRO_CLIFF_U cliff; // 4
	struct MIRO_WHEELS_U2 wheels; // 4
	struct MIRO_SPKR_U spkr; // 4
	struct MIRO_MIC_U mic; // 1004
	struct MIRO_USR_I2C_U usr_i2c; // 40
};

struct MIRO_P1B_D
{
	struct MIRO_P1_PARS pars; // 64

	struct MIRO_PLATFORM_D platform_d; // 32
	struct MIRO_SERVO_D lift; // 8
	struct MIRO_SERVO_D yaw; // 8
	struct MIRO_SERVO_D droop; // 8
	struct MIRO_SERVO_D wag; // 8
	struct MIRO_SPKR_D spkr; // 264
	struct MIRO_USR_I2C_D usr_i2c; // 64
	struct MIRO_MIC_D mic; // 4

	//	make structures same size
	uint8_t __packing[660];
};

//	confirm size for multiple compilers
#define MIRO_P1B_SIZE (1120)
__PREPROC_ASSERT_SIZE_IS(MIRO_P1B_U, MIRO_P1B_SIZE)
__PREPROC_ASSERT_SIZE_IS(MIRO_P1B_D, MIRO_P1B_SIZE)



////////////////////////////////////////////////////////////////
//
//	MIRO_P1H
//
////////////////////////////////////////////////////////////////

struct MIRO_P1H_U
{
	struct MIRO_PLATFORM_U platform_u; // 32
	struct MIRO_LOG_U log; // 32
	struct MIRO_SERVO_U pitch; // 8
	struct MIRO_SONAR_U sonar; // 8
	struct MIRO_IMU_U imu; // 20
	struct MIRO_TOUCH_U touch; // 4

	//	make structures same size
	uint8_t __packing[32];
};

struct MIRO_P1H_D
{
	struct MIRO_P1_PARS pars; // 64

	struct MIRO_PLATFORM_D platform_d; // 32
	struct MIRO_SERVO_D pitch; // 8
	struct MIRO_SERVO_D eye[2]; // 16 (8 x 2)
	struct MIRO_SERVO_D ear[2]; // 16 (8 x 2)
};

//	confirm size for multiple compilers
#define MIRO_P1H_SIZE (136)
__PREPROC_ASSERT_SIZE_IS(MIRO_P1H_U, MIRO_P1H_SIZE)
__PREPROC_ASSERT_SIZE_IS(MIRO_P1H_D, MIRO_P1H_SIZE)



#endif



