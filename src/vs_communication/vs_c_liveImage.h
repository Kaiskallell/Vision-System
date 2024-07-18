/**
 * @copyright Copyright (c) 2019 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 *
 * @brief liveimage headerfile
 */
#ifndef VS_C_LIVEIMAGE_H
#define VS_C_LIVEIMAGE_H

class LiveImage
{
  private:
	size_t nSize;

  public:
	bool processLiveImageCamera(std::string s, std::string& r);
	bool processLiveImageVisionSystem(std::string s, std::string& r);

	// setter for size --  send message
	void setMessageSize(size_t nNewSize) { this->nSize = nNewSize; }
	// getter for size - send message
	size_t getMessageSize() { return this->nSize; }
};

#endif
