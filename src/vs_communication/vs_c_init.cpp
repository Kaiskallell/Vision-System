/*
 * Created on Tue Nov 26 2019
 *
 * Copyright (c) 2019 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 */
#include "vs_c_init.h"

#include <iostream>

#include "vs.h"
#include "vs_messaging.h"

// protofiles
#include "Init.pb.h"

using namespace VMS::VisionSystem;  // protofiles

int Init::processInit(const std::string& sMessageBuffer, std::string& result)
{
	// can not extract/parse message
	InitRequest initRequest;
	if (!initRequest.ParseFromString(sMessageBuffer))
	{
		return InitParsing;
	}

	m_kLogger->debug("initRequest.major_version() = {}", initRequest.major_version());
	m_kLogger->debug("initRequest.minor_version() = {}", initRequest.minor_version());

	// check communication version
	if ((initRequest.major_version() != VS_COMMUNICATION_ST_MAJOR_VERSION)
	    || (initRequest.minor_version() != VS_COMMUNICATION_ST_MINOR_VERSION))
	{
		m_kLogger->warn("VS_COMMUNICATION_ST_VERSION= {}.{} ",
		                VS_COMMUNICATION_ST_MAJOR_VERSION,
		                VS_COMMUNICATION_ST_MINOR_VERSION);
		m_kLogger->warn("initRequest version= {}.{}", initRequest.major_version(), initRequest.minor_version());
		m_kLogger->warn("Version requested by VMS does not match VS version!");
		return InitVersion;
	}

	// check version for robot type
	InitResponseAck initResponseAck;
	if (nRobotType == InitResponseAck_VSHardwareType_TYP_TOG519)
	{
		initResponseAck.set_hardware_type(InitResponseAck_VSHardwareType_TYP_TOG519);
		initResponseAck.set_major_version(VS_COMMUNICATION_ST_MAJOR_VERSION);
		initResponseAck.set_minor_version(VS_COMMUNICATION_ST_MINOR_VERSION);
		initResponseAck.set_patch_level(VS_COMMUNICATION_ST_PATCH_VERSION);
	}
	else
	{
		m_kLogger->error("Wrong Robot Type");
		return InitRobotTypeWrong;
	}

	setMessageSize(initResponseAck.ByteSize());
	initResponseAck.SerializeToString(&result);

	return InitOk;
}
