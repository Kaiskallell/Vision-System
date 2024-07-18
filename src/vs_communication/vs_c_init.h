/*
 * Copyright (c) 2019 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 */
#ifndef __VS_C_INIT_H__
#define __VS_C_INIT_H__

#include <iostream>

#include "logging.h"

class Init
{
  private:
	size_t nSize;
	int nRobotType;
	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("VSCommInitVMS");

  public:
	explicit Init(int nNewRobotType)
	{
		nRobotType = nNewRobotType;
		nSize = 0;
	}

	enum eInitReturnValues
	{
		InitRobotTypeWrong = -3,
		InitVersion = -2,
		InitParsing = -1,
		InitOk = 0,
	};

	int processInit(const std::string& sMessageBuffer, std::string& result);

	// setter robot type
	void setRobotType(int nNewRobotType) { this->nRobotType = nNewRobotType; }
	int getRobotType() { return this->nRobotType; }

	// setter for size --  send message
	void setMessageSize(size_t nNewSize) { this->nSize = nNewSize; }
	// getter for size - send message
	size_t getMessageSize() { return this->nSize; }
};

#endif
