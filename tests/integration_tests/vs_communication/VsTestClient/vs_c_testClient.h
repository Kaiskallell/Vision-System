/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef COMM_CLIENT_TEST_H
#define COMM_CLIENT_TEST_H

#include <boost/process.hpp>
#include <memory>

#include "gtest/gtest.h"
#include "redis++.h"
#include "vs.h"
#include "vs_c_tcpConnector.h"
#include "vs_messaging.h"

namespace bp = boost::process;

class CommunicationTest : public ::testing::Test
{
  protected:
	CommunicationTest();

	/**
	 * @brief sends _requestbuffer via stream
	 *
	 * @param stream destination for the data which will be sent
	 */
	void request(const std::shared_ptr<TCPStream>& stream);

	/**
	 * @brief recieives data from stream. save it in _messageBodyBuffer
	 *
	 * @param stream  source of the received data
	 */
	void receive(const std::shared_ptr<TCPStream>& stream);

	/**
	 * @brief Set the Up Stream object
	 *
	 * @param server adress of the vision system
	 * @param port
	 * @return std::shared_ptr<TCPStream>
	 */
	std::shared_ptr<TCPStream> setUpStream(const char* server, int port) const;

	/**
	 * @brief prints out some error messages when there is a error status
	 *
	 * @param nStatus ACK or NACK
	 * @param cMessage contains information of the error in case of NACK
	 */
	void checkStatus(const int nStatus, const char* cMessage) const;

	char _requestBuffer[VS_MESSAGE_BUFFER_SIZE];
	char _responseBuffer[VS_MESSAGE_BUFFER_SIZE];
	char _messageBodyBuffer[VS_MESSAGE_BUFFER_SIZE];

	vs_message_header_request _header_request;
	vs_message_header_response _header_response;
	int _sequenceNumber = 0;
	sw::redis::Redis _redis;
	bp::child _process_child;
};

#endif  // COMM_CLIENT_TEST_H
