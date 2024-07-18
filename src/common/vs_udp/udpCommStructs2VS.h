/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef COMM_STRUCTS_TO_VS_H
#define COMM_STRUCTS_TO_VS_H

#include <cstdint>

#include "VSCommDefinitions.h"
#include "vs.h"  // versions

// KC --> Vision-System

////////////////////////////////////////

static constexpr size_t kTc3ToVsBufferSize = 60;

struct sVersionKC_VS
{
	uint32_t uMainVersion = VS_UDP_MAJOR_VERSION;
	uint32_t uMinorVersion = VS_UDP_MINOR_VERSION;
	uint32_t uRevision = VS_UDP_PATCH_VERSION;
};

struct sHeaderKC_VS
{
	sVersionKC_VS sVersion;
	uint32_t uWatchdogCounter;
};

struct sCommandKC_VS
{
	eCommandVSComm eActualCommandVSComm;
};

struct sCameraTriggerKC_VS
{
	uint64_t uImageNumber;
	uint32_t uActValRotEncoderConveyor;
	uint32_t uYear;
	uint32_t uMonth;
	uint32_t uDay;
	uint32_t uHour;
	uint32_t uMinute;
	uint32_t uSecond;
	uint32_t uMilliSecond;
};

////////////////////////////////////////

struct Tc3ToVsDatagram
{
	sHeaderKC_VS sHeader;
	sCommandKC_VS sCommand;
	sCameraTriggerKC_VS sCameraTrigger;
};
#endif
