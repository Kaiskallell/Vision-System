/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef __VS_C_VERSION__
#define __VS_C_VERSION__

#define TO_STRING_EXP(v) #v
#define TO_STRING(v) TO_STRING_EXP(v)

#define VS_C_VERSION_MAJOR 0
#define VS_C_VERSION_MINOR 3
#define VS_C_VERSION_REVISION 4

#define VS_C_VERSION              \
	TO_STRING(VS_C_VERSION_MAJOR) \
	"." TO_STRING(VS_C_VERSION_MINOR) "." TO_STRING(VS_C_VERSION_REVISION)

#endif  // __VS_C_VERSION__