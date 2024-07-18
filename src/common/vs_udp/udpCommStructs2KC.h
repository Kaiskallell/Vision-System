/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef COMM_STRUCTS_TO_KC_H
#define COMM_STRUCTS_TO_KC_H

#include <cstdint>

#include "VSCommDefinitions.h"
#include "vs.h"  // versions
// KC <-- Vision-System

////////////////////////////////////////
struct sVersionVS_KC
{
	uint32_t uMainVersion = VS_UDP_MAJOR_VERSION;
	uint32_t uMinorVersion = VS_UDP_MINOR_VERSION;
	uint32_t uRevision = VS_UDP_PATCH_VERSION;
};

struct sHeaderVS_KC
{
	sVersionVS_KC sVersion;
	uint32_t uWatchdogCounter = 0;
	uint32_t uAreaNumber = 0;
};

struct sCommandVS_KC
{
	eCommandVSComm eActualCommandVSComm;
};

struct sProductsSituationVS_KC
{
	eProductsSituationVSComm eActualProductsSituationVSComm;
	uint32_t classId = 0;
};

////////////////////////////////////////

struct VsToTc3Datagram
{
	sHeaderVS_KC sHeader;
	sCommandVS_KC sCommand;
	sProductsSituationVS_KC sProductsSituation;
};

#endif  // CAMERACOMMIN_H
