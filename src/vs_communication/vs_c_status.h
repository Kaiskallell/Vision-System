/**
 * @copyright Copyright (c) 2020 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * @brief send object to robot
 *
 */
#ifndef __VS_C_STATUS_H__
#define __VS_C_STATUS_H__

#include <bitset>
#include <iostream>
#include <memory>

#include "GetStatus.pb.h"
#include "dbFacade/dbFacade.h"
#include "logging.h"
#include "redis++.h"
#include "vs.h"

using namespace sw::redis;          // database
using namespace VMS::VisionSystem;  // protofiles

// bit definition of ea_image_output
#define EA_OUTPUT_BIT_ACTIVE 0
// offset definition area bits
#define EA_OUTPUT_AREA_OFFSET 3
// when ST needs informations about Area
#define EA_OUTPUT_AREA_INFO_REQUEST 16

class GetStatus
{
  private:
	std::shared_ptr<db::DbFacade> m_dbFacade;
	size_t nSize;

	std::vector<uint64_t> m_frameIdOldAreas;

	uint32_t m_kObjectsRequestedBit = 0xFF00;  // aka object request
	std::vector<int> m_kObjectReq;
	std::vector<int> m_requestedAreas;

	static void printDataWhichAreSend(const GetStatusResponseAck& getStatusResponseAck);
	void addObjectsFromArea(GetStatusResponseAck& getStatusResponseAck, const size_t areaNumber);
	bool checkAreasToBeUpdated(const uint32_t digitalIoCtrlToVs);
	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("GetStatus");

  public:
	GetStatus();
	int processGetStatus(const std::string& sMessageBufferIn, std::string& sMessageBufferOut);

	// setter
	void setMessageSize(size_t nNewSize) { this->nSize = nNewSize; }

	// getter
	size_t getMessageSize() { return this->nSize; }

	enum eGetStatusVsVMSReturnValues
	{
		GetStatusVsVmsNoRequest = -5,
		GetStatusVsVmsNoFrameCouter = -4,
		GetStatusVsVmsNoNewFrame = -3,
		GetStatusVsVmsEmptyDB = -2,
		GetStatusVsVmsParsing = -1,
		GetStatusVsVmsOk = 0,
	};
};
#endif
