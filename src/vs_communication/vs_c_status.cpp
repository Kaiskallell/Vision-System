/**
 * @copyright Copyright (c) 2020 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * @brief create string (serialized object) with protobuf with all the products of a scene
 *
 */

#include "vs_c_status.h"

#include <unistd.h>

#include <fstream>

#include "projectPaths.h"
#include "vs_messaging.h"

GetStatus::GetStatus()
{
	m_dbFacade = std::make_shared<db::DbFacade>(utils::getProjectRootDir() / "config/visionSystemConfig.json");
	for (int i = 0; i < 8; i++)
	{
		m_kObjectReq.push_back(0x100 * (int)pow(2, i));
		m_frameIdOldAreas.push_back(0);
	}
}

void GetStatus::printDataWhichAreSend(const GetStatusResponseAck& getStatusResponseAck)
{
	// there is at least one pickable product on the belt
	// print the pose coordinates which are send to L&R
	for (int j = 0; j < getStatusResponseAck.area_infos_size(); ++j)
	{
		uint64_t areaNumber = getStatusResponseAck.area_infos(j).area_number();
		uint64_t frameUid = getStatusResponseAck.area_infos(j).frame_uid();
		for (int i = 0; i < getStatusResponseAck.area_infos(j).object_infos_size(); i++)
		{
			int32_t coord_x = getStatusResponseAck.area_infos(j).object_infos(i).coord_x();
			int32_t coord_y = getStatusResponseAck.area_infos(j).object_infos(i).coord_y();
			int32_t coord_z = getStatusResponseAck.area_infos(j).object_infos(i).coord_z();
			int32_t roll = getStatusResponseAck.area_infos(j).object_infos(i).angle_roll();
			int32_t pitch = getStatusResponseAck.area_infos(j).object_infos(i).angle_pitch();
			int32_t yaw = getStatusResponseAck.area_infos(j).object_infos(i).angle_yaw();

			uint32_t product_id = getStatusResponseAck.area_infos(j).object_infos(i).object_id();  // aka classId
			uint64_t product_uid = getStatusResponseAck.area_infos(j).object_infos(i).object_uid();
			m_kLogger->debug(
			    "areaNumber = {}, frame_uid = {},  product_uid = {}, product_id = {}, coord_x = {}, coord_y= "
			    "{}, coord_z = {}, "
			    "roll = {}, pitch "
			    "= {}, yaw = {}",
			    areaNumber,
			    frameUid,
			    product_uid,
			    product_id,
			    coord_x,
			    coord_y,
			    coord_z,
			    roll,
			    pitch,
			    yaw);
		}
	}
}

void GetStatus::addObjectsFromArea(GetStatusResponseAck& getStatusResponseAck, const size_t areaNumber)
{
	// get object infos
	std::string serializedSceneInfos = m_dbFacade->getSerializedObjInfosFromDB(areaNumber);
	if (serializedSceneInfos.empty())
	{
		return;
	}

	GetStatusResponseAck sceneInfosConveyor;
	sceneInfosConveyor.ParseFromString(serializedSceneInfos);

	// alternative if above doesnt work
	VMS::VisionSystem::AreaInfo* areaInfo = getStatusResponseAck.add_area_infos();
	for (int i = 0; i < sceneInfosConveyor.area_infos_size(); ++i)
	{
		if (sceneInfosConveyor.area_infos(i).frame_uid()
		    == m_frameIdOldAreas.at(areaNumber - 1))  // To prevent from gost picks
		{
			continue;
		}
		m_frameIdOldAreas.at(areaNumber - 1) = sceneInfosConveyor.area_infos(i).frame_uid();
		VMS::VisionSystem::AreaInfo* areaInfo = getStatusResponseAck.add_area_infos();
		*areaInfo = sceneInfosConveyor.area_infos(i);
	}
}

bool GetStatus::checkAreasToBeUpdated(const uint32_t digitalIoCtrlToVs)
{
	m_requestedAreas.clear();
	if (!static_cast<bool>(m_kObjectsRequestedBit & digitalIoCtrlToVs))  // get specific bit
	{
		m_dbFacade->setExtReq(false, 7); // This should be done to detect the high flag later
		m_dbFacade->setExtReq(false, 8); // This should be done to detect the high flag later
		return false;  // no area to be updated
	}
	else
	{
		for (int i = 1; i <= m_kObjectReq.size(); i++)
		{
			if (i == 5 || i == 6)  // area 5 and 6 are not needed now
			{
				continue;
			}
			else
			{
				if (static_cast<bool>(m_kObjectReq.at(i - 1) & digitalIoCtrlToVs))
				{
					m_requestedAreas.emplace_back(i);
					if (i == 7 || i == 8)  // for area 7 and 8, we need first to trigger an image, that is why we set a
					// request of the image
					{
						m_dbFacade->setExtReq(true, i);
					}
				}
				else
				{
					if (i == 7 || i == 8)
					{
						m_dbFacade->setExtReq(false, i); // This should be done to detect the high flag later
					}
				}
			}
		}
		return true;  // at least one area needs to be updated
	}
}

int GetStatus::processGetStatus(const std::string& sMessageBufferIn, std::string& sMessageBufferOut)
{
	sMessageBufferOut = "";
	// processing request from VMS
	GetStatusRequest getStatusRequest;
	GetStatusResponseAck getStatusResponseAck;
	if (!getStatusRequest.ParseFromString(sMessageBufferIn))
	{
		m_kLogger->debug("Could not get ParseFromString in GetStatus");
		return GetStatusVsVmsParsing;  // can not extract/parse message
	}
	uint32_t digitalIoCtrlToVs = getStatusRequest.digital_io_ctrl_to_vs();
	if (!checkAreasToBeUpdated(digitalIoCtrlToVs))  // detecting if we need to update one area
	{
		setMessageSize(getStatusResponseAck.ByteSize());
		getStatusResponseAck.SerializeToString(&sMessageBufferOut);
		return GetStatusVsVmsNoRequest;
	}
	for (int i = 0; i < m_requestedAreas.size(); ++i)
	{
		addObjectsFromArea(getStatusResponseAck, m_requestedAreas.at(i));
	}

	if (getStatusResponseAck.area_infos_size() == 0)
	{
		setMessageSize(getStatusResponseAck.ByteSize());
		getStatusResponseAck.SerializeToString(&sMessageBufferOut);
		// m_kLogger->error("Could not get objectInfos in GetPose");
		return GetStatusVsVmsEmptyDB;  // false leads to stop of robot (VMS) and we need to manually restart
	}
	printDataWhichAreSend(getStatusResponseAck);  // only needed if we want to print it

	setMessageSize(getStatusResponseAck.ByteSize());
	getStatusResponseAck.SerializeToString(&sMessageBufferOut);

	return GetStatusVsVmsOk;
}
