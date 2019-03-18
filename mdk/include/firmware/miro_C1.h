#ifndef INC_MIRO_C1_H
#define INC_MIRO_C1_H



#include "miro_P0.h"



////////////////////////////////    IMU

#define MIRO_I2C_ADDR_IMU               0xD0
#define MIRO_I2C_ADDR_SC7A20			0b00110000



////////////////////////////////    LEDS

#define MIRO_I2C_ADDR_LED               0xC0



////////////////////////////////    SWITCHES

#define MIRO_I2C_ADDR_SW                0x40



////////////////////////////////    TOUCH

//	MIRO1 touch boards
#define MIRO_I2C_ADDR_BODY_TOUCH		0x92
#define MIRO_I2C_ADDR_HEAD_TOUCH		0xA2

//	MIRO2 touch boards
#define MIRO_I2C_ADDR_BODY_CAP1114		0x50
#define MIRO_I2C_ADDR_HEAD_CAP1114		0x50



////////////////////////////////    LEGACY ILLUM

#define MIRO_I2C_ADDR_ILLUM0            0x90
#define MIRO_I2C_ADDR_ILLUM1            0x8E



#endif



