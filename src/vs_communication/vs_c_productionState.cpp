/**
 * @copyright Copyright (c) 2018 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 *
 * @brief production state class
 */

#include "vs_c_productionState.h"

#include "projectPaths.h"
// protofiles
#include "StartProduction.pb.h"

using namespace VMS::VisionSystem;  // protofiles

ProductionState::ProductionState()
{
	m_dbFacade = std::make_shared<db::DbFacade>(utils::getProjectRootDir() / "config/visionSystemConfig.json");
}

int ProductionState::processStartProduction(const std::string& sMessageBuffer, std::string& result)
{
	StartProductionRequest startProductionRequest;
	// can not extract/parse message
	if (!startProductionRequest.ParseFromString(sMessageBuffer))
	{
		return StartProductionParsing;
	}

	int nFromatID = startProductionRequest.program_id();

	int nFormatIDDatabase = m_dbFacade->getFormatId();
	// id equals db or debugging
	if ((nFormatIDDatabase == nFromatID) || (nFromatID == INT32_MAX))
	{
		StartProductionResponseAck startProductionResponseAck;
		setMessageSize(startProductionResponseAck.ByteSize());
		startProductionResponseAck.SerializeToString(&result);

		return StartProductionOk;
	}
	return StartProductionWrongID;

	m_kLogger->error("Format query not in DB");

	return StartProductionIDNotInDB;
}