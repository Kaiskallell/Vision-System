/*
 * Copyright (c) 2019 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 */
#ifndef __VS_C_VERSIONCHECKVSPS_H__
#define __VS_C_VERSIONCHECKVSPS_H__

#include <iostream>

#include "logging.h"

class VersionCheckVsPs
{
  private:
	size_t nSize = 0;
	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("VersionCheckVsPs");

  public:
	enum eVersionCheckVsPsReturnValues
	{
		VersionCheckVsPsVersion = -2,
		VersionCheckVsPsParsing = -1,
		VersionCheckVsPsOk = 0,
	};

	int processVersionCheckVsPs(std::string sMessageBuffer, std::string& result);

	// setter for size --  send message
	void setMessageSize(size_t nNewSize) { this->nSize = nNewSize; }
	// getter for size - send message
	size_t getMessageSize() { return this->nSize; }
};

#endif  // __VS_C_VERSIONCHECKVSPS_H__
