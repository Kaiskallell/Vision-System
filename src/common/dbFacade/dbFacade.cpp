/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "dbFacade.h"

#include <fstream>
#include <iostream>

#include "GetStatus.pb.h"
#include "json.hpp"
namespace db
{
DbFacade::DbFacade(const fs::path& dbConfigFilePath)
{
	m_config = readConfig(dbConfigFilePath);
	m_redis = std::make_shared<sw::redis::Redis>(m_config->databaseLink);

	// All keys which are use in the database should be defined here
	m_frameCounterKey1 = "processing:" + std::to_string(m_kVisionSystemId) + ":frameCounter:1";
	m_frameCounterKey2 = "processing:" + std::to_string(m_kVisionSystemId) + ":frameCounter:2";
	m_frameCounterKey3 = "processing:" + std::to_string(m_kVisionSystemId) + ":frameCounter:3";
	m_frameCounterKey4 = "processing:" + std::to_string(m_kVisionSystemId) + ":frameCounter:4";
	m_frameCounterKey7 = "processing:" + std::to_string(m_kVisionSystemId) + ":frameCounter:7";
	m_frameCounterKey8 = "processing:" + std::to_string(m_kVisionSystemId) + ":frameCounter:8";

	m_workChainChangedPickKey = "processing:" + std::to_string(m_kVisionSystemId) + ":program:workchain:changed";
	m_formatKey = "format:" + std::to_string(m_kVisionSystemId) + ":active";
	m_classIdMappingKey = "format:classIdMappingList";
	m_productIdMappingKey = "format:productIdMappingList";
	m_imprintNameMappingKey = "format:imprintNameMappingList";
	m_sceneInfosKeyArea1 = "sceneInfosKeyArea1";
	m_sceneInfosKeyArea2 = "sceneInfosKeyArea2";
	m_sceneInfosKeyArea3 = "sceneInfosKeyArea3";
	m_sceneInfosKeyArea4 = "sceneInfosKeyArea4";
	m_sceneInfosKeyArea7 = "sceneInfosKeyArea7";  // for extensions like classification
	m_sceneInfosKeyArea8 = "sceneInfosKeyArea8";  // for extensions like classification
	m_extReqKey7 = "extReqKey7";
	m_extReqKey8 = "extReqKey8";

	m_shutDownProcesses = "shutDownProcesses";
	m_errorCodeKey = "errorCode";
}

void DbFacade::flushDB() { m_redis->flushall(); writeErrorCodeToDB(0);}

std::shared_ptr<DbConfig> DbFacade::readConfig(const fs::path& dbConfigFilePath) const
{
	std::ifstream ifs(dbConfigFilePath.string());
	if (!ifs.good())
	{
		throw std::runtime_error("cannot open config file! dbConfigFilePath = " + dbConfigFilePath.string());
	}

	nlohmann::json jsonFile = nlohmann::json::parse(ifs);
	std::shared_ptr<DbConfig> config = std::make_shared<DbConfig>();
	config->databaseLink = jsonFile.at("databaseLink").get<std::string>();
	return config;
}

void DbFacade::serializeProductInfosToDB(const std::vector<VSPose>& posesInRobotCoordinates,
                                         const std::vector<int>& classIds,
                                         const int32_t rotEncoderValue,
                                         const uint64_t frameId,
                                         const uint32_t areaNumber,
                                         const ObjectType objType)
{
	VMS::VisionSystem::GetStatusResponseAck infosOfScene;
	int errorNumber = getErrorCodeFromDB();
	if (errorNumber != 0)
	{
		infosOfScene.set_error_number(errorNumber);
		infosOfScene.set_error_class(91);
	}
	VMS::VisionSystem::AreaInfo* areaInfo = infosOfScene.add_area_infos();
	areaInfo->set_area_number(areaNumber);  // sometimes also called TransportStrecke
	if (areaNumber == 1)
	{
		areaInfo->set_frame_uid(frameId);
	}
	else if (areaNumber == 2)
	{
		constexpr uint64_t kMagicOffsetForFrameId = 300000;  // TODO(mkellil): make this better
		areaInfo->set_frame_uid(frameId + kMagicOffsetForFrameId);
	}
	else if (areaNumber == 7)
	{
		constexpr uint64_t kMagicOffsetForFrameId = 600000;  // TODO(mkellil): make this better
		areaInfo->set_frame_uid(frameId + kMagicOffsetForFrameId);
	}
	else if (areaNumber == 8)
	{
		constexpr uint64_t kMagicOffsetForFrameId = 900000;  // TODO(mkellil): make this better
		areaInfo->set_frame_uid(frameId + kMagicOffsetForFrameId);
	}

	this->writeFrameCounterToDb(frameId, areaNumber);
	for (int i = 0; i < posesInRobotCoordinates.size(); ++i)
	{
		VMS::VisionSystem::ObjectInfo* objectInfo = areaInfo->add_object_infos();

		VSPose pose = posesInRobotCoordinates.at(i);
		objectInfo->set_coord_x(convertTo10thMM(pose.x));
		objectInfo->set_coord_y(convertTo10thMM(pose.y));
		objectInfo->set_coord_z(convertTo10thMM(pose.z));
		// angle
		objectInfo->set_angle_roll(convertTo10thGrad(pose.roll));
		objectInfo->set_angle_pitch(convertTo10thGrad(pose.pitch));
		objectInfo->set_angle_yaw(convertTo10thGrad(pose.yaw));
		objectInfo->set_encoder_value(rotEncoderValue);

		// common product settings
		if (areaNumber == 1)
		{
			objectInfo->set_sequence_number(m_objectUid);
			objectInfo->set_object_uid(m_objectUid);  // TODO(mkellil): same like above?
			objectInfo->set_object_id(classIds.at(i));
			objectInfo->set_update_count(0);
		}
		else if (areaNumber == 2)
		{
			constexpr uint64_t kMagicOffsetForProdUID = 500000;  // TODO(mkellil): make this better
			objectInfo->set_sequence_number(m_objectUid + kMagicOffsetForProdUID);
			objectInfo->set_object_uid(m_objectUid + kMagicOffsetForProdUID);  // TODO(mkellil): same like above?
			objectInfo->set_object_id(classIds.at(i));
			objectInfo->set_update_count(0);
		}
		else if (areaNumber == 7)
		{
			constexpr uint64_t kMagicOffsetForProdUID = 1000000;  // TODO(mkellil): make this better
			objectInfo->set_sequence_number(m_objectUid + kMagicOffsetForProdUID);
			objectInfo->set_object_uid(m_objectUid + kMagicOffsetForProdUID);  // TODO(mkellil): same like above?
			objectInfo->set_object_id(classIds.at(i));
			objectInfo->set_update_count(0);
		}
		else if (areaNumber == 8)
		{
			constexpr uint64_t kMagicOffsetForProdUID = 1500000;  // TODO(mkellil): make this better
			objectInfo->set_sequence_number(m_objectUid + kMagicOffsetForProdUID);
			objectInfo->set_object_uid(m_objectUid + kMagicOffsetForProdUID);  // TODO(mkellil): same like above?
			objectInfo->set_object_id(classIds.at(i));
			objectInfo->set_update_count(0);
		}

		objectInfo->set_object_type(VMS::VisionSystem::ObjectType::Product);
		objectInfo->set_update_count(0);

		++m_objectUid;
	}

	std::string serializedPosesOfScene = "";
	infosOfScene.SerializeToString(&serializedPosesOfScene);

	if (areaNumber == 1)
	{
		m_redis->set(m_sceneInfosKeyArea1, serializedPosesOfScene);
	}
	if (areaNumber == 2)
	{
		m_redis->set(m_sceneInfosKeyArea2, serializedPosesOfScene);
	}
	if (areaNumber == 7)
	{
		m_redis->set(m_sceneInfosKeyArea7, serializedPosesOfScene);
	}
	if (areaNumber == 8)
	{
		m_redis->set(m_sceneInfosKeyArea8, serializedPosesOfScene);
	}
}

void DbFacade::serializeTrayInfosToDB(const std::vector<VSPose>& posesInRobotCoordinates,
                                      const std::vector<int>& classIds,
                                      const int32_t rotEncoderValue,
                                      const uint64_t frameId,
                                      const std::vector<uint64_t>& trackingIds,
                                      const uint32_t areaNumber)
{
	VMS::VisionSystem::GetStatusResponseAck infosOfScene;
	int errorNumber = getErrorCodeFromDB();
	if (errorNumber != 0)
	{
		infosOfScene.set_error_number(errorNumber);
		infosOfScene.set_error_class(91);
	}
	VMS::VisionSystem::AreaInfo* areaInfo = infosOfScene.add_area_infos();
	areaInfo->set_area_number(areaNumber);  // sometimes also called visionSystemId or TransportStrecke
	if (areaNumber == 3)
	{
		constexpr uint64_t kMagicOtherBeltOffsetForFrameId = 600000;  // TODO(aschaefer): make this better
		areaInfo->set_frame_uid(frameId + kMagicOtherBeltOffsetForFrameId);
	}
	else if (areaNumber == 4)
	{
		constexpr uint64_t kMagicOtherBeltOffsetForFrameId = 900000;  // TODO(aschaefer): make this better
		areaInfo->set_frame_uid(frameId + kMagicOtherBeltOffsetForFrameId);
	}
	this->writeFrameCounterToDb(frameId, areaNumber);
	for (int i = 0; i < posesInRobotCoordinates.size(); ++i)
	{
		VMS::VisionSystem::ObjectInfo* objectInfo = areaInfo->add_object_infos();

		VSPose pose = posesInRobotCoordinates.at(i);
		objectInfo->set_coord_x(convertTo10thMM(pose.x));
		objectInfo->set_coord_y(convertTo10thMM(pose.y));
		objectInfo->set_coord_z(convertTo10thMM(pose.z));
		// angle
		objectInfo->set_angle_roll(convertTo10thGrad(pose.roll));
		objectInfo->set_angle_pitch(convertTo10thGrad(pose.pitch));
		objectInfo->set_angle_yaw(convertTo10thGrad(pose.yaw));

		objectInfo->set_encoder_value(
		    rotEncoderValue
		    + i);  // "+i" is for L&R to make sure that they are different detections. Shoud be fixed somewhen.

		// common tray settings
		if (areaNumber == 3)
		{
			constexpr uint64_t kMagicOtherBeltOffsetForProdUID = 1000000;  // TODO(aschaefer): make this better
			objectInfo->set_sequence_number(trackingIds.at(i) + kMagicOtherBeltOffsetForProdUID);
			objectInfo->set_object_uid(trackingIds.at(i)
			                           + kMagicOtherBeltOffsetForProdUID);  // TODO(aschaefer): same like above?
			objectInfo->set_object_id(classIds.at(i));                      // back or font side of product
		}
		else if (areaNumber == 4)
		{
			constexpr uint64_t kMagicOtherBeltOffsetForProdUID = 1500000;  // TODO(aschaefer): make this better
			objectInfo->set_sequence_number(trackingIds.at(i) + kMagicOtherBeltOffsetForProdUID);
			objectInfo->set_object_uid(trackingIds.at(i)
			                           + kMagicOtherBeltOffsetForProdUID);  // TODO(aschaefer): same like above?
			objectInfo->set_object_id(classIds.at(i));                      // back or font side of product
		}

		objectInfo->set_object_type(VMS::VisionSystem::ObjectType::Tray);
		objectInfo->set_update_count(1);
	}

	std::string serializedPosesOfScene = "";
	infosOfScene.SerializeToString(&serializedPosesOfScene);

	if (areaNumber == 3)
	{
		m_redis->set(m_sceneInfosKeyArea3, serializedPosesOfScene);
	}
	if (areaNumber == 4)
	{
		m_redis->set(m_sceneInfosKeyArea4, serializedPosesOfScene);
	}
}

std::string DbFacade::getSerializedObjInfosFromDB(const int areaNumber)
{
	sw::redis::OptionalString sceneInfos;
	if (areaNumber == 1)
	{
		sceneInfos = m_redis->get(m_sceneInfosKeyArea1);
		m_redis->del(m_sceneInfosKeyArea1);  // delete to avoid buffering and gost picks
	}
	if (areaNumber == 2)
	{
		sceneInfos = m_redis->get(m_sceneInfosKeyArea2);
		m_redis->del(m_sceneInfosKeyArea2);  // delete to avoid buffering and gost picks
	}
	if (areaNumber == 3)
	{
		sceneInfos = m_redis->get(m_sceneInfosKeyArea3);
		m_redis->del(m_sceneInfosKeyArea3);  // delete to avoid buffering and gost places
	}
	if (areaNumber == 4)
	{
		sceneInfos = m_redis->get(m_sceneInfosKeyArea4);
		m_redis->del(m_sceneInfosKeyArea4);  // delete to avoid buffering and gost places
	}
	if (areaNumber == 7)
	{
		sceneInfos = m_redis->get(m_sceneInfosKeyArea7);
		m_redis->del(m_sceneInfosKeyArea7);  // delete to avoid buffering and gost features extractions
	}
	if (areaNumber == 8)
	{
		sceneInfos = m_redis->get(m_sceneInfosKeyArea8);
		m_redis->del(m_sceneInfosKeyArea8);  // delete to avoid buffering and gost features extractions
	}
	return std::move(sceneInfos.value());
}

void DbFacade::deleteSerializedObjInfosFromDB()
{
	m_redis->del(m_sceneInfosKeyArea1);  // delete to avoid buffering and gost picks
	m_redis->del(m_sceneInfosKeyArea2);  // delete to avoid buffering and gost picks
	m_redis->del(m_sceneInfosKeyArea3);  // delete to avoid buffering and gost places
	m_redis->del(m_sceneInfosKeyArea4);  // delete to avoid buffering and gost places
	m_redis->del(m_sceneInfosKeyArea7);  // delete to avoid buffering and gost features extractions
	m_redis->del(m_sceneInfosKeyArea8);  // delete to avoid buffering and gost features extractions
}

bool DbFacade::checkIfFormatChanged()
{
	sw::redis::OptionalString workChainChanged = m_redis->get(m_workChainChangedPickKey);
	if (!workChainChanged)  // if redis key does exist (std::optional)
	{
		throw std::runtime_error("could not retrieve value from database key 'm_workChainChangedPickKey'= "
		                         + m_workChainChangedPickKey);
	}

	if (stoi(workChainChanged.value()))  // check if it is 1 aka true
	{
		return true;
	}
	return false;
}

void DbFacade::setFormatChanged(bool workChainChanged)
{
	if (workChainChanged)
	{
		m_redis->set(m_workChainChangedPickKey, "1");
	}
	else
	{
		m_redis->set(m_workChainChangedPickKey, "0");
	}
}

void DbFacade::writeFormatToDB(const uint32_t formatId) { m_redis->set(m_formatKey, std::to_string(formatId)); }

void DbFacade::writeErrorCodeToDB(const int errorCode) { m_redis->set(m_errorCodeKey, std::to_string(errorCode)); }

int DbFacade::getErrorCodeFromDB()
{
	sw::redis::OptionalString sErrorCode = m_redis->get(m_errorCodeKey);
	if(!sErrorCode)
	{
		throw std::runtime_error("could not retrieve value from database key 'm_errorCodeKey'= " + m_errorCodeKey);
	}
	writeErrorCodeToDB(0);
	return stoi(sErrorCode.value());
}

uint32_t DbFacade::getFormatId()
{
	sw::redis::OptionalString sFormatId = m_redis->get(m_formatKey);
	if (!sFormatId)
	{
		throw std::runtime_error("could not retrieve value from database key 'm_formatKey'= " + m_formatKey);
	}
	return stoi(sFormatId.value());
}

void DbFacade::writeImprintMappingToDB(const ImprintMappings imprintMappings)
{
	// delete old mapping
	m_redis->del(m_classIdMappingKey);
	m_redis->del(m_productIdMappingKey);
	m_redis->del(m_imprintNameMappingKey);

	// write new mapping to db lists
	for (size_t i = 0; i < imprintMappings.size(); ++i)
	{
		uint32_t classId = imprintMappings.Get(i).class_id();
		uint32_t productId = imprintMappings.Get(i).product_id();
		std::string imprintName = imprintMappings.Get(i).imprint_name();
		m_redis->lpush(m_classIdMappingKey, std::to_string(classId));
		m_redis->lpush(m_productIdMappingKey, std::to_string(productId));
		m_redis->lpush(m_imprintNameMappingKey, imprintName);
	}
}

std::vector<ImprintMap> DbFacade::getImprintMappingFromDB()
{
	std::vector<std::string> classIdStrs;
	std::vector<std::string> productIdStrs;
	std::vector<std::string> imprintNames;

	// get lists from data base
	m_redis->lrange(m_classIdMappingKey, 0, -1, std::back_inserter(classIdStrs));
	m_redis->lrange(m_productIdMappingKey, 0, -1, std::back_inserter(productIdStrs));
	m_redis->lrange(m_imprintNameMappingKey, 0, -1, std::back_inserter(imprintNames));

	// convert lists to ImprintMap object
	std::vector<ImprintMap> mappingTable;
	for (size_t i = 0; i < classIdStrs.size(); ++i)
	{
		ImprintMap mapping;
		mapping.classId = stoi(classIdStrs[i]);
		mapping.productId = stoi(productIdStrs[i]);
		mapping.imprintName = imprintNames[i];
		mappingTable.push_back(mapping);
	}
	return mappingTable;
}

uint64_t DbFacade::getFrameCounter(size_t areaNumber)
{
	// Get Frame ID
	sw::redis::OptionalString sFrameID;
	if (areaNumber == 1)
	{
		sFrameID = m_redis->get(m_frameCounterKey1);
	}
	if (areaNumber == 2)
	{
		sFrameID = m_redis->get(m_frameCounterKey2);
	}
	if (areaNumber == 3)
	{
		sFrameID = m_redis->get(m_frameCounterKey3);
	}
	if (areaNumber == 4)
	{
		sFrameID = m_redis->get(m_frameCounterKey4);
	}
	if (areaNumber == 7)
	{
		sFrameID = m_redis->get(m_frameCounterKey7);
	}
	if (areaNumber == 8)
	{
		sFrameID = m_redis->get(m_frameCounterKey8);
	}

	if (!sFrameID)
	{
		throw std::runtime_error("could not retrieve value from database key of area: " + std::to_string(areaNumber));
	}

	return stoi(sFrameID.value());
}

void DbFacade::writeFrameCounterToDb(const uint64_t frameCounter, const size_t areaNumber)
{
	if (areaNumber == 1)
	{
		m_redis->set(m_frameCounterKey1, std::to_string(frameCounter));
	}
	if (areaNumber == 2)
	{
		m_redis->set(m_frameCounterKey2, std::to_string(frameCounter));
	}
	if (areaNumber == 3)
	{
		m_redis->set(m_frameCounterKey3, std::to_string(frameCounter));
	}
	if (areaNumber == 4)
	{
		m_redis->set(m_frameCounterKey4, std::to_string(frameCounter));
	}
	if (areaNumber == 7)
	{
		m_redis->set(m_frameCounterKey7, std::to_string(frameCounter));
		;
	}
	if (areaNumber == 8)
	{
		m_redis->set(m_frameCounterKey8, std::to_string(frameCounter));
		;
	}
}

uint32_t checkIfClassIdsAreInImprintMapping(const std::vector<int>& classIds,
                                            const std::vector<db::ImprintMap>& imprintMappingTable,
                                            const std::shared_ptr<spdlog::logger> logger)
{
	uint32_t errorNumber = 0;  // 0 means no error
	for (size_t i = 0; i < classIds.size(); ++i)
	{
		uint32_t classId = classIds[i];
		// check if classId from classificator neural net is the desired product (is in imprint mapping table)
		auto ret = std::find_if(imprintMappingTable.begin(),
		                        imprintMappingTable.end(),
		                        [&classId](const db::ImprintMap& mapping) { return mapping.classId == classId; });
		if (ret == imprintMappingTable.end())  // if not found
		{
			logger->error("Classificator found classId = {} which is not in imprint mapping", classId);
			if (imprintMappingTable.empty())
			{
				logger->error("imprintMappingTable is empty!");
			}
			// print out which mapping is available
			for (size_t j = 0; j < imprintMappingTable.size(); ++j)
			{
				logger->error("{}: {} --> {}",
				              imprintMappingTable[j].imprintName,
				              imprintMappingTable[j].classId,
				              imprintMappingTable[j].productId);
			}
			errorNumber =
			    0x1008;  // one detected Product is not in mapping. errorNumber needs to be between 0x1007 and 0x2000.
		}
	}
	// return errorNumber;
	return 0;  // not needed at the moment
}

void DbFacade::setShutDownProcesses(bool shutdown)
{
	if (shutdown)
	{
		m_redis->set(m_shutDownProcesses, "true");
	}
	else
	{
		m_redis->set(m_shutDownProcesses, "false");
	}
}

bool DbFacade::getShutDownProcesses()
{
	sw::redis::OptionalString shutDownStr = m_redis->get(m_shutDownProcesses);
	if (!shutDownStr)
	{
		throw std::runtime_error("could not retrieve value from database key 'm_shutDownProcesses'= "
		                         + m_shutDownProcesses);
	}
	if (shutDownStr.value() == "true")
	{
		return true;
	}
	return false;
}

bool DbFacade::getExtReq(const int areaNumber)
{
	sw::redis::OptionalString extReqKeyStr;
	if (areaNumber == 7)
	{
		extReqKeyStr = m_redis->get(m_extReqKey7);
	}
	else if (areaNumber == 8)
	{
		extReqKeyStr = m_redis->get(m_extReqKey8);
	}
	else
	{
		throw std::runtime_error("wrong areaNumber is used for extensions'= " + areaNumber);
	}

	if (!extReqKeyStr)
	{
		throw std::runtime_error("could not retrieve value from database key 'extReqKeyStr' for areaNumber = "
		                         + areaNumber);
	}
	if (extReqKeyStr.value() == "true")
	{
		return true;
	}
	return false;
}

void DbFacade::setExtReq(bool extReq, const int areaNumber)
{
	std::string extReqKey;
	if (areaNumber == 7)
	{
		extReqKey = m_extReqKey7;
	}
	else if (areaNumber == 8)
	{
		extReqKey = m_extReqKey8;
	}
	else
	{
		throw std::runtime_error("wrong areaNumber is used for extensions'= " + areaNumber);
	}
	if (extReq)
	{
		m_redis->set(extReqKey, "true");
	}
	else
	{
		m_redis->set(extReqKey, "false");
	}
}

void writePosesToDB(const std::shared_ptr<db::DbFacade> dbFacade,
                    const std::vector<VSPose>& posesInRobotCoordinates,
                    const std::vector<int>& classIds,
                    const uint64_t frameNumber,
                    const uint32_t rotEncoderValue,
                    const size_t areaNumber,
                    const std::vector<db::ImprintMap>& imprintMappingTable,
                    const std::shared_ptr<spdlog::logger> m_kLogger)
{
	// writing the poses to data base so vsCommunication can access it and send it to VMS
	dbFacade->serializeProductInfosToDB(posesInRobotCoordinates,
	                                    classIds,
	                                    rotEncoderValue,
	                                    frameNumber,
	                                    areaNumber,
	                                    db::ObjectType::product);
}

// this is called in constructor of pickconveyor.cpp etc.
void setupDBObjectForDetection(std::shared_ptr<db::DbFacade> dbFacade,
                               std::vector<db::ImprintMap>& imprintMappingTable,
                               const fs::path& visionSystemConfigPath,
                               const int formatId,
                               const size_t areaNumber,
                               const std::shared_ptr<spdlog::logger> kLogger)
{
	try
	{
		// for interaction with the database we use a facade class for better encapsulation
		dbFacade = std::make_shared<db::DbFacade>(visionSystemConfigPath);
	}
	catch (const std::exception& e)
	{
		kLogger->error(e.what());
		kLogger->error("Cannot create dbFacade");
		exit(-1);
	}
	dbFacade->writeFormatToDB(formatId);  // needed in getProductionState for VMS otherwise sending error to VMS
	dbFacade->writeFrameCounterToDb(0, areaNumber);  // needed in GetStatus for VMS otherwise sending error to VMS
	if(areaNumber == 7 || areaNumber == 8)
	{
		dbFacade->setExtReq(false, areaNumber); //needed in classification extension
	}
	imprintMappingTable =
	    dbFacade
	        ->getImprintMappingFromDB();  // needed to translate the classId of a product to productId which VMS needs

	try
	{
		dbFacade->getShutDownProcesses();  // check if key exists in DB.
		                                         // In case detectPickable was called alone and without missionControl
		                                         // which usuallly initializes this key.
	}
	catch (const std::exception& e)
	{
		// init  if it doesnt exists
		dbFacade->setShutDownProcesses(false);  // we want to start it
	}
}

}  // namespace db
