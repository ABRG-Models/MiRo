/**
	This file defines the message exchange protocol for the C3
	USB link. It comprises a message format which is the same in
	both directions, each message containing a P2 message as
	defined in miro_P2.h, wrapped for transport over USB.

	A C3 message comprises a header, a body (the P2 message),
	and a footer. Headers and footers contain signature words
	(for the detection of corruption), release, and message
	size, which can take any value.

	However, hard-coded buffers are used in some
	implementations, so large messages will overflow these and
	cause failure. Thus, this file defines maximum message
	sizes, and implementations should fail gracefully if these
	are exceeded.

	NB: Because the downstream end (P2) has different resource
	limits from the upstream end (P3), and because the nature of
	the data passed in each direction is different, the maximum
	message size in each direction is, accordingly, different.

	The complete structure of a C3 message is:

		C3 header
		{
			manifest
			data (0 or more bytes)
		}
		C3 footer
*/

#ifndef INC_MIRO_C3_H
#define INC_MIRO_C3_H



#include "miro_P2.h"



////////////////////////////////////////////////////////////////
//
//		C3 HEADER & FOOTER
//
////////////////////////////////////////////////////////////////

/**
	Maximum message sizes upstream (P2->P3) and downstream
	(P3->P2). The largest thing we send upstream is a JPEG
	block, which necessarily fits in 32kB/2 (SRAM3). For
	downstream, it's a matter of not using RAM downstream
	unless necessary, without any overflow events.
*/
#define MIRO_C3_MAX_MSG_SIZE_UP (128 * 1024)
#define MIRO_C3_MAX_MSG_SIZE_DN 4096

/**
	Message header and footer signature words.
*/
#define MIRO_C3_SIG_HEADER 0x4F52494D
#define MIRO_C3_SIG_FOOTER 0x4D49524F

/**
	C3 transport header (for upstream and downstream messages).
*/
struct MIRO_C3_H
{
	/**
		Set to MIRO_C3_SIG_HEADER.
	*/
	uint32_t signature;

	/**
		Release of the entity that packaged this message for
		transport.
	*/
	uint16_t release;

	/**
		Message ID is implementation defined. It should be
		unique amongst messages that are sent nearby in time.

		Typically, ID is set equal to the number of messages
		sent so far by the implementation.
	*/
	uint16_t msg_id;

	/**
		Size in bytes of message (not including this header or
		corresponding footer).

		This value is the same as the value "msg_size" in the
		message manifest for many messages. However, messages
		may be padded for transport (compressed messages, in
		particular, are padded to maintain alignment). In such
		cases, this value may be larger than that given in the
		manifest.
	*/
	uint32_t msg_size;

	/**
		The result of summing the header, not including
		this member, using uint32_t overflow arithmetic.
	*/
	uint32_t checksum;
};

#define MIRO_C3_H_SIZE 16
__PREPROC_ASSERT_SIZE(MIRO_C3_H)

/**
	C3 transport footer (for upstream and downstream messages).
*/
struct MIRO_C3_F
{
	/**
		Matches the id in MIRO_C3_H.
	*/
	uint16_t msg_id;

	//	align / padding for possible future extension
	uint16_t __padding[5];

	/**
		Set to MIRO_C3_SIG_FOOTER.
	*/
	uint32_t signature;
};

#define MIRO_C3_F_SIZE 16
__PREPROC_ASSERT_SIZE(MIRO_C3_F)



#endif



