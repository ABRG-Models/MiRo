
#ifndef INC_MIRO_MSG_H
#define INC_MIRO_MSG_H



/** A MIRO message, either with or without transport wrappers.
*/
typedef uint8_t* MIRO_MSG;
typedef const uint8_t* CONST_MIRO_MSG;

/** \addtogroup manifest
	Message manifest
	@{
*/

/** Messages passed between processing centres in MIRO all
	start with a manifest, as defined below, followed by a data
	block of zero or more bytes known as the payload. Thus, all
	MIRO messages have the structure:

	{ manifest, payload }

	which is described, together, as the "message body".

	NB: The manifest is distinct from the transport wrapper
	(header and footer) in which a message is packaged before
	sending over a physical transport link. Transport wrappers
	are described elsewhere for specific physical links. A
	wrapped message has the structure:

	{ header, manifest, payload, footer }
*/
struct MIRO_MANIFEST
{
	/** Unique identifier of release of sending entity. This is
		a bit-packed date code in the following form:

		[1:unreleased][6:year-2000][4:month][5:day]

		On build, the build date is stored as the release code,
		with the upper bit set to 1 to indicate that this is not
		a release code, but a build date. On release, the code
		is replaced with the tag of the release, which is the
		date the release was made (therefore, only one release
		can be made in any one day).
	*/
	uint16_t release;

	/** Entity MIRO_ENTITY_* that created this message.
	*/
	uint8_t originator;

	/** Zero or more manifest flags.
	*/
	uint8_t flags;

	/** Size of the message body, comprising this manifest and
		the block of data that follows it.
	*/
	uint32_t msg_size;

	/** Message type, any MIRO_MSG_TYPE* constant.
	*/
	uint16_t msg_type;

	/** Micro data object facilitating short messages.
	*/
	uint16_t user_data;

	/** Message identity; usage is implementation-specific.
	*/
	uint16_t msg_id;

	/** If this message was evoked in response to one coming the
		other way, the id of that first message is put here.
	*/
	uint16_t in_reply_to;
};

/** Construct release */
#define __MIRO_RELEASE(y, m, d) (y << 9 | m << 5 | d)

/** Manifest size */
#define MIRO_MANIFEST_SIZE 16

//	invalid release
#define MIRO_RELEASE_INVALID 0xBDBD

#define __MIRO_MANIFEST_DEFAULT(manifest) \
	do { \
	(manifest).release = 0; /* filled in by sending layer */ \
	(manifest).originator = MIRO_ENTITY_UNKNOWN; \
	(manifest).flags = 0; \
	(manifest).msg_size = MIRO_MANIFEST_SIZE; \
	(manifest).msg_type = MIRO_MSG_TYPE_NULL; \
	(manifest).user_data = 0; \
	(manifest).msg_id = 0; \
	(manifest).in_reply_to = 0; \
	} while(0)

#define __MIRO_MANIFEST_SET(manifest, type) \
	do { \
	__MIRO_MANIFEST_DEFAULT(manifest); \
	(manifest).originator = IMAGE_ENTITY; \
	(manifest).msg_type = type; \
	} while(0)

/** @} */

__PREPROC_ASSERT_SIZE(MIRO_MANIFEST)

/** \addtogroup releases
	Software release.
	@{
*/

/** @} */

/** \addtogroup entities
	System entities.
	@{
*/
#define MIRO_ENTITY_UNKNOWN			0
#define MIRO_ENTITY_P1				0x10
#define MIRO_ENTITY_P1h				0x11
#define MIRO_ENTITY_P1a				0x12
#define MIRO_ENTITY_P1b				0x13
#define MIRO_ENTITY_P1p				0x14
#define MIRO_ENTITY_P2				0x20
#define MIRO_ENTITY_P3				0x30
#define MIRO_ENTITY_P4				0x40
#define MIRO_ENTITY_MDK				0x50
#define MIRO_ENTITY_USER			0x80
/** @} */

/** \addtogroup groups
	Message type groups
	@{
*/
#define MIRO_MSG_TYPE_GROUP_MASK	0xFF00
#define MIRO_MSG_TYPE_GROUP_COMMON	0x0000
#define MIRO_MSG_TYPE_GROUP_P1		0x1000
#define MIRO_MSG_TYPE_GROUP_P2		0x2000
#define MIRO_MSG_TYPE_GROUP_P3		0x3000

/** @} */

/** \addtogroup types
	Message types
	@{
*/

/** A message. */
#define MIRO_MSG_TYPE_NULL			(MIRO_MSG_TYPE_GROUP_COMMON + 0x00)

/** Consists only of a manifest. Intended for the return
	of the result code of an operation in "user_data". */
#define MIRO_MSG_TYPE_RESULT		(MIRO_MSG_TYPE_GROUP_COMMON + 0x01)

/** Consists only of a manifest. "user_data" can take any
	value the caller likes. */
#define MIRO_MSG_TYPE_PING			(MIRO_MSG_TYPE_GROUP_COMMON + 0x02)

/** Consists only of a manifest. Sent in response to PING.
	"user_data" should be set to the bit-wise inversion of
	"user_data" in the PING message. */
#define MIRO_MSG_TYPE_PONG			(MIRO_MSG_TYPE_GROUP_COMMON + 0x03)

/** Consists of a manifest followed by a null-terminated string
	of characters from one of the log streams. The value of
	"user_data" is a MIRO_ENTITY_* constant indicating the source. */
#define MIRO_MSG_TYPE_LOG			(MIRO_MSG_TYPE_GROUP_COMMON + 0x04)

/** @} */

/** \addtogroup Manifest_Flags
	Message manifest flags
	@{
*/

/** The operation completed successfully */
#define MIRO_RESULT_OK					0

/** No valid manifest was found in the message */
#define MIRO_RESULT_NO_MANIFEST			1

/** The size specified in the manifest did not match the
	size specified in the transport wrapper */
#define MIRO_RESULT_SIZE_MISMATCH		2

/** The message was not of the size expected for its type */
#define MIRO_RESULT_INVALID_MSG_SIZE	3

/** The parameters of the operation were not acceptable */
#define MIRO_RESULT_INVALID_PARAMS		4

/** @} */



#endif




