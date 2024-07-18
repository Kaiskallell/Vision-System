/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#pragma once

/////////////////////////////////////////

enum eCommandVSComm : uint32_t
{
	eCommandVSComm_NONE = 0x1000,
	eCommandVSComm_SYNC = 0x1001,
	eCommandVSComm_TAKEIMAGE = 0x1002,
	eCommandVSComm_UPDATEPRODUCTSSITUATION = 0x1003
};

enum eProductsSituationVSComm : uint32_t
{
	eProductsSituationVSComm_AVAILABLE = 0x0001,
	eProductsSituationVSComm_NOTAVAILABLE = 0x0002,
	eProductsSituationVSComm_BADPRODUCT = 0x0003,
	eProductsSituationVSComm_AVAILABLE_SECONDTRAY = 0x0004
};
