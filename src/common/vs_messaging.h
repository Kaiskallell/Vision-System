/*
 * Created on Sun Aug 05 2018
 *
 * Copyright (c) 2018 Gerhard Schubert GmbH - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 *
 */
#ifndef VS_MESSAGING_H
#define VS_MESSAGING_H

#define VS_MESSAGE_BUFFER_SIZE 10240

#include <stdint.h>

//** definition of command identfier vs_communication to ST*/
typedef enum
{
	// ST
	VS_NONE = 0x00000000,
	VS_INIT_REQUEST = 0x00010001,
	// currently not used
	// VS_TIME_SYNC = 0x00010002,
	// VS_REGISTER_STATUS_INFO = 0x00010003,
	VS_GET_STATUS = 0x00010004,
	VS_START_PRODUCTION = 0x00010005,
	// currently not used
	// VS_STOP_PRODUCTION = 0x00010006,
	// VS_RESET = 0x00010007,
	// VS_ACKNOWLEDGE_ERROR = 0x00010008,

	// VS
	VS_APP_DATA_GET = 0x00020001,
	VS_APP_DATA_SET = 0x00020002,
	VS_COUNTER = 0x00020003,
	VS_COUNTER_RESET = 0x00020004,
	VS_PROGRAM_AVAILABLE = 0x00020005,
	VS_PROGRAM = 0x00020006,
	VS_CALIBRATION_START = 0x00020007,
	VS_CALIBRATION_END = 0x00020008,
	VS_CALIBRATION_FRAME = 0x00020009,
	// VS_SYSTEM_STATUS = 0x00020010,
	VS_MARKER_POSITION_GET = 0x00020011,
	VS_MARKER_POSITION_SET = 0x00020012,
	VS_MARKER_POSITION_START = 0x00020013,
	VS_MARKER_POSITION_END = 0x00020014,

	VS_CAMERA_SYSTEMS_GET = 0x00020015,
	VS_CAMERA_LIVE_IMAGE = 0x00020016,
	VS_VISION_SYSTEM_LIVE_IMAGE = 0x00020017,
	// VS_PROGRAM_LOADED_RESPONSE = 0x00020018,
	VS_APP_MESSAGE_GET = 0x00020019,
	VS_FORMAT = 0x00020020,
	VS_FORMAT_AVAILABLE = 0x00020021,
	VS_FORMAT_LOADED = 0x00020022,

	VS_VERSION_CHECK = 0x00020023,

} eCommandIdentifier;

//*definitions of messages to ST*/
typedef enum
{
	NACK = 0x00000000,
	ACK = 0x00000001,
	BUSY = 0x00000002,
	ERROR = 0xFFFFFFFF,
} eResponseStatus;

#define VS_MESSAGE_STARTIDENTIFIER 0x10101010

// Header response message
// 20byte
class vs_message_header_response
{
  public:
	const uint32_t nStartIdentifier = VS_MESSAGE_STARTIDENTIFIER;
	uint32_t nSequneceNumber;
	eCommandIdentifier nCommandIdentifier;
	eResponseStatus nStatus;
	uint32_t nLength;
};

// Header request message
// 16byte
class vs_message_header_request
{
  public:
	const uint32_t nStartIdentifier = VS_MESSAGE_STARTIDENTIFIER;
	uint32_t nSequneceNumber;
	eCommandIdentifier nCommandIdentifier;
	uint32_t nLength;
};

#endif
