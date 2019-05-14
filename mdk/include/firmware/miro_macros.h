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
	This header provides common macros we use widely.
*/

#ifndef INC_MIRO_MACROS_H
#define INC_MIRO_MACROS_H



#define __BIT(b) (((uint32_t)1) << (b))

#define __SET_FIELD(s, key, value) s.key = (value)

#define __UNUSED_PARAM(param) { (void)param; }

#define __CONSTRAIN(x, a, b) \
do { \
	if ((x) < a) { x = a; } \
	if ((x) > b) { x = b; } \
} while(0)

#define __MAX(a, b) ((a) > (b) ? (a) : (b))

#define __MIN(a, b) ((a) < (b) ? (a) : (b))

#define __MIN_MAX(x, a, b) ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))

#define __ABS(a) (((a) < 0) ? (-(a)) : (a))

#define __SIGN(a) (((a) < 0) ? (-1) : (+1))

#define __DEG2RAD(x) ((x) * (float)(M_PI / 180.0))

#define __RAD2DEG(x) ((x) * (float)(180.0 / M_PI))

#define __MEMZERO(addr, bytes) \
do { \
	uint8_t* _p = (uint8_t*) (addr); \
	uint32_t _n; \
	for (_n=0; _n<(bytes); _n++) \
		_p[_n] = 0; \
} while(0)

#define __OBJZERO(obj) \
do { \
	uint8_t* _p = (uint8_t*) (&obj); \
	uint32_t _n; \
	for (_n=0; _n<(sizeof(obj)); _n++) \
		_p[_n] = 0; \
} while(0)

#define __MEMSET(addr, value, bytes) \
do { \
	uint8_t* p = (uint8_t*) (addr); \
	uint32_t _n; \
	for (_n=0; _n<(bytes); _n++) \
		p[_n] = value; \
} while(0)

#define __MEMCPY(dst, src, bytes) \
do { \
	uint8_t* pd = (uint8_t*) (dst); \
	uint8_t* ps = (uint8_t*) (src); \
	uint32_t _n; \
	for (_n=0; _n<(bytes); _n++) \
		pd[_n] = ps[_n]; \
} while(0)

#define __MEMCPY32(dst, src, bytes) \
do { \
	uint32_t* pd = (uint32_t*) (dst); \
	uint32_t* ps = (uint32_t*) (src); \
	uint32_t _n; \
	for (_n=0; _n<(bytes>>2); _n++) \
		pd[_n] = ps[_n]; \
} while(0)

#define __PREPROC_ASSERT( condition, message ) \
    typedef char assert_failed_ ## message [ (condition) ? 1 : -1 ];

#define __PREPROC_ASSERT_SIZE(obj) \
	__PREPROC_ASSERT( sizeof(struct obj) == obj ## _SIZE, obj ## _SIZE)

#define __PREPROC_ASSERT_SIZE_IS(obj, sz) \
	__PREPROC_ASSERT( sizeof(struct obj) == sz, obj ## _SIZE)



#endif




