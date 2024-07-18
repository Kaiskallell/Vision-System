/*
 * Created on Tue Nov 26 2019
 *
 * Copyright (c) 2019 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 */
#ifndef VS_C_STATE_H
#define VS_C_STATE_H

#include "dbFacade.h"
#include "logging/logging.h"

class ProductionState
{
  private:
	size_t nSize;
	std::shared_ptr<db::DbFacade> m_dbFacade;
	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("ProductionState");

  public:
	ProductionState();
	enum eStatusReturnValues
	{
		StartProductionIDNotInDB = -3,
		StartProductionWrongID = -2,
		StartProductionParsing = -1,
		StartProductionOk = 0,
	};
	int processStartProduction(const std::string& sMessageBuffer, std::string& result);
	// currently not used
	// bool processStopProduction(std::string s, std::string &r);
	// bool processResetProduction(std::string s, std::string &r);

	// setter for size --  send message
	void setMessageSize(size_t nNewSize) { this->nSize = nNewSize; }
	// getter for size - send message
	size_t getMessageSize() { return this->nSize; }
};

#endif
