/**
	@section COPYRIGHT
	Copyright (C) 2019 Consequential Robotics Ltd
	
	@section AUTHOR
	Consequential Robotics http://consequentialrobotics.com
	
	@section LICENSE
	For a full copy of the license agreement, and a complete
	definition of "The Software", see LICENSE in the MDK root
	directory.
	
	Subject to the terms of this Agreement, Consequential
	Robotics grants to you a limited, non-exclusive, non-
	transferable license, without right to sub-license, to use
	"The Software" in accordance with this Agreement and any
	other written agreement with Consequential Robotics.
	Consequential Robotics does not transfer the title of "The
	Software" to you; the license granted to you is not a sale.
	This agreement is a binding legal agreement between
	Consequential Robotics and the purchasers or users of "The
	Software".
	
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
	This header provides constants used in the firmware.
*/

#ifndef INC_MIRO_FIRMWARE_CONSTANTS_H
#define INC_MIRO_FIRMWARE_CONSTANTS_H



////////////////////////////////////////////////////////////////
//	TIME

//	MIRO platform internal tick time
#define MIRO_PLATFORM_TICK_HZ (50)
#define MIRO_PLATFORM_TICK_SEC ((1.0f) / MIRO_PLATFORM_TICK_HZ)
#define MIRO_PLATFORM_TICK_USEC (1000000 / MIRO_PLATFORM_TICK_HZ)

//	MIRO microsecond timer rolls over at a human-readable point
#define MIRO_TIMER_ROLLOVER_SEC (1000)
#define MIRO_TIMER_ROLLOVER_USEC (MIRO_TIMER_ROLLOVER_SEC * 1000 * 1000)

//	audio sample rates
#define MIRO_MIC_SAMPLE_RATE						20000
#define MIRO_SPKR_SAMPLE_RATE						8000



////////////////////////////////////////////////////////////////
//	FLAGS

//	(non-persistent) downstream flags (per-tick)
#define MIRO_PLATFORM_D_FLAG_DISABLE_ILLUM			__BIT(0)
#define MIRO_PLATFORM_D_FLAG_DISABLE_STATUS_LEDS	__BIT(1)
#define MIRO_PLATFORM_D_FLAG_DISABLE_SONAR			__BIT(2)
#define MIRO_PLATFORM_D_FLAG_DISABLE_CLIFF			__BIT(3)
#define MIRO_PLATFORM_D_FLAG_DISABLE_TOUCH_HEAD		__BIT(4)
#define MIRO_PLATFORM_D_FLAG_DISABLE_TOUCH_BODY		__BIT(5)
#define MIRO_PLATFORM_D_FLAG_DISABLE_CAMERAS		__BIT(6)
#define MIRO_PLATFORM_D_FLAG_DISABLE_BODY_MICS		__BIT(7)
#define MIRO_PLATFORM_D_FLAG_DISABLE_HEAD_MICS		__BIT(8)
#define MIRO_PLATFORM_D_FLAG_DISABLE_SPEAKER		__BIT(9)
#define MIRO_PLATFORM_D_FLAG_DISABLE_SPI_USR		__BIT(10)
#define MIRO_PLATFORM_D_FLAG_DISABLE_DOMAIN_REPORTS	__BIT(11)
#define MIRO_PLATFORM_D_FLAG_DISABLE_WHEEL_CONTROL	__BIT(12)
#define MIRO_PLATFORM_D_FLAG_DISABLE_OPTO			__BIT(13)
#define MIRO_PLATFORM_D_FLAG_DISABLE_SERVO_POWER	__BIT(14)
#define MIRO_PLATFORM_D_FLAG_DISABLE_SERVO_SENSORS	__BIT(15)
#define MIRO_PLATFORM_D_FLAG_DISABLE_SERVO_PWM0		__BIT(16)
#define MIRO_PLATFORM_D_FLAG_DISABLE_DIP_SWITCH		__BIT(17)
#define MIRO_PLATFORM_D_FLAG_DISABLE_IMU_HEAD		__BIT(18)
#define MIRO_PLATFORM_D_FLAG_DISABLE_IMU_BODY		__BIT(19)
#define MIRO_PLATFORM_D_FLAG_DISABLE_I2C_FAIL_WARN	__BIT(20)
#define MIRO_PLATFORM_D_FLAG_DISABLE_CLIFF_REFLEX	__BIT(21)
#define MIRO_PLATFORM_D_FLAG_DISABLE_KIN_IDLE		__BIT(22)
#define MIRO_PLATFORM_D_FLAG_DISABLE_WHEELS			__BIT(23)
#define MIRO_PLATFORM_D_FLAG_DISABLE_TRANSLATION	__BIT(24)
#define MIRO_PLATFORM_D_FLAG_DISABLE_SPKR_MUTE		__BIT(25)

//	if this is set, the passed flags are stored (else they
//	expire after a timeout)
#define MIRO_PLATFORM_D_FLAG_PERSISTENT				__BIT(31)

//	persistent upstream flags (per-session)
#define MIRO_PLATFORM_U_FLAG_READY					__BIT(0)
#define MIRO_PLATFORM_U_FLAG_PARAMETRISED			__BIT(1)
#define MIRO_PLATFORM_U_FLAG_HEAD_JP2				__BIT(2)
#define MIRO_PLATFORM_U_FLAG_HEAD_MOSI_HIGH			__BIT(3)

//	non-persistent upstream flags (per-tick)
#define MIRO_PLATFORM_U_FLAG_SPI_FAIL				__BIT(16)
#define MIRO_PLATFORM_U_FLAG_I2C_P1H_FAIL			__BIT(17)
#define MIRO_PLATFORM_U_FLAG_I2C_P1A_FAIL			__BIT(18)
#define MIRO_PLATFORM_U_FLAG_I2C_P1B_FAIL			__BIT(19)
#define MIRO_PLATFORM_U_FLAG_WHEELS_STALL			__BIT(20)
#define MIRO_PLATFORM_U_FLAG_LOW_BATTERY			__BIT(21)



#endif




