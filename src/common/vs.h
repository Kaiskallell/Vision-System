/**
 * @copyright Copyright (c) 2018 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 *
 * @brief main header file
 */

#ifndef VISION_SYSTEM_H
#define VISION_SYSTEM_H

//! @param define default communication server for communication with VMS which acts as client
#define VS_C_DEFAULT_IP "127.0.0.1"
#define VS_C_SERVER_PORT 53001

#define TO_STRING_EXP(v) #v
#define TO_STRING(v) TO_STRING_EXP(v)

#define VS_P_VERSION_MAJOR 2
#define VS_P_VERSION_MINOR 8
#define VS_P_VERSION_REVISION 0

#define VS_P_VERSION              \
	TO_STRING(VS_P_VERSION_MAJOR) \
	"." TO_STRING(VS_P_VERSION_MINOR) "." TO_STRING(VS_P_VERSION_REVISION)

#define VS_VERSION "3.0.0"

// Protocol version has to be defined with L&R from common repo
#define VS_COMMUNICATION_ST_MAJOR_VERSION (1)
#define VS_COMMUNICATION_ST_MINOR_VERSION (1)
#define VS_COMMUNICATION_ST_PATCH_VERSION (0)

#define VS_COMMUNICATION_PS_VERSION_MAJOR (2)
#define VS_COMMUNICATION_PS_VERSION_MINOR (0)
#define VS_COMMUNICATION_PS_VERSION_PATCH (0)

// udp version for communication with tc3
#define VS_UDP_MAJOR_VERSION (3)
#define VS_UDP_MINOR_VERSION (0)
#define VS_UDP_PATCH_VERSION (0)

#endif  // VISION_SYSTEM_H
