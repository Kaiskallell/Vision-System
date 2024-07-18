/*
 * Copyright (c) 2019 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 */
#include "vs_c_versionCheckVsPs.h"

#include <iostream>

#include "vs.h"
#include "vs_messaging.h"

// protofiles
#include "VersionCheckVsPs.pb.h"

using namespace VMS::VisionSystem;  // protofiles

int VersionCheckVsPs::processVersionCheckVsPs(std::string sMessageBuffer, std::string& result)
{
	// Tell PS which Version we have even if versions are incompatible
	VersionResponse versionResponse;
	versionResponse.set_major_version(VS_COMMUNICATION_PS_VERSION_MAJOR);
	versionResponse.set_minor_version(VS_COMMUNICATION_PS_VERSION_MINOR);
	versionResponse.set_patch_level(VS_COMMUNICATION_PS_VERSION_PATCH);

	setMessageSize(versionResponse.ByteSize());
	versionResponse.SerializeToString(&result);

	// can not extract/parse message
	VersionRequest versionRequest;
	if (!versionRequest.ParseFromString(sMessageBuffer))
	{
		return VersionCheckVsPsParsing;
	}

	// check communication version
	if ((versionRequest.major_version() != VS_COMMUNICATION_PS_VERSION_MAJOR)
	    || (versionRequest.minor_version() != VS_COMMUNICATION_PS_VERSION_MINOR))
	{
		m_kLogger->debug("Version requested by PS does not match VS version!");
		m_kLogger->debug("versionRequest.major_version() = {}", versionRequest.major_version());
		m_kLogger->debug("versionRequest.minor_version() = {}", versionRequest.minor_version());
		return VersionCheckVsPsVersion;
	}

	return VersionCheckVsPsOk;
}
