/**
 * @copyright Copyright (c) 2019 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 *
 * @brief headerfile marker class
 */
#ifndef VS_C_MARKER_H
#define VS_C_MARKER_H

#include "json.hpp"

using namespace nlohmann;

class MarkerClass
{
  private:
	size_t nSize;

  public:
	bool processMarkerStart(std::string s, std::string& r);
	bool processMarkerEnd(std::string s, std::string& r);
	bool processMarkerGet(std::string s, std::string& r);
	bool processMarkerSet(std::string s, std::string& r);

	// setter for size --  send message
	void setMessageSize(size_t nNewSize) { this->nSize = nNewSize; }
	// getter for size - send message
	size_t getMessageSize() { return this->nSize; }
};
#endif
