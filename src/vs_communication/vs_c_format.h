/**
 * @copyright Copyright (c) 2020 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 *
 * @brief format headerfile
 *
 */
#ifndef __VS_C_FORMAT_H__
#define __VS_C_FORMAT_H__

#include <iostream>

#include "dbFacade/dbFacade.h"
#include "redis++.h"
#include "vs.h"

class Format
{
  private:
	size_t nSize;
	std::unique_ptr<db::DbFacade> m_dbFacade;
	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("Format");

	void writeFormatIdToDisk(const uint32_t formatId);

  public:
	Format();
	enum eFormatReturnValues
	{
		FormatUnknownDBEntry = -5,
		FormatUnknown = -4,
		FormatDirNotFound = -3,
		FormatFileNotFound = -2,
		FormatParsing = -1,
		FormatOk = 0,
	};
	bool processFormatAvailable(const std::string& sMessageBuffer, std::string& result);
	bool processFormatLoaded(const std::string& sMessageBuffer, std::string& result);
	int processFormat(const std::string& sMessageBuffer, std::string& result);

	// setter for size --  send message
	void setMessageSize(size_t nNewSize) { this->nSize = nNewSize; }
	// getter for size - send message
	size_t getMessageSize() { return this->nSize; }
};

#endif
