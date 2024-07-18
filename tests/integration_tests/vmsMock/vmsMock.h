/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef VMS_MOCK_H
#define VMS_MOCK_H

#include <memory>

#include "logging/logging.h"
#include "vs.h"
#include "vs_c_tcpConnector.h"
#include "vs_messaging.h"

class VmsMock
{
  public:
	VmsMock();
	void checkVersionNumber();
	uint64_t makeGetStatusRequest(uint32_t digitalIoCtrlToVs);

  protected:
	/**
	 * @brief sends _requestbuffer via stream
	 *
	 * @param stream destination for the data which will be sent
	 */
	void request(std::shared_ptr<TCPStream> stream);

	/**
	 * @brief recieives data from stream. save it in _messageBodyBuffer
	 *
	 * @param stream  source of the received data
	 */
	void receive(std::shared_ptr<TCPStream> stream);

	/**
	 * @brief Set the Up Stream object
	 *
	 * @param server adress of the vision system
	 * @param port
	 * @return std::unique_ptr<TCPStream>
	 */
	std::shared_ptr<TCPStream> setUpStream(const char* server, int port) const;

	/**
	 * @brief prints out some error messages when there is a error status
	 *
	 * @param nStatus ACK or NACK
	 * @param cMessage contains information of the error in case of NACK
	 */
	void checkStatus(const unsigned int, const char* cMessage) const;

	char m_requestBuffer[VS_MESSAGE_BUFFER_SIZE];
	char m_responseBuffer[VS_MESSAGE_BUFFER_SIZE];
	char m_messageBodyBuffer[VS_MESSAGE_BUFFER_SIZE];

	vs_message_header_request m_header_request;
	vs_message_header_response m_header_response;
	int m_sequenceNumber = 0;

	std::shared_ptr<TCPStream> m_stream;
	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("vmsMock");
};

#endif  // VMS_MOCK_H
