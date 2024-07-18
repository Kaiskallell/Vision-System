/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "vmsMock.h"

#include <unistd.h>

#include <experimental/filesystem>
#include <iostream>

#include "Format.pb.h"
#include "GetStatus.pb.h"
#include "Init.pb.h"
#include "Nack.pb.h"
#include "projectPaths.h"

VmsMock::VmsMock()
{
	while (!m_stream)
	{
		try
		{
			m_stream = setUpStream(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);
		}
		catch (const std::exception& e)
		{
			m_kLogger->error(e.what());
			sleep(1);  // avoid cluttering logs/cmd line
		}
	}
}

void VmsMock::checkVersionNumber()
{
	m_stream = setUpStream(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);
	VMS::VisionSystem::InitRequest initRequest;
	// create and initialize protobuf object
	initRequest.set_major_version(1);
	initRequest.set_minor_version(0);
	initRequest.set_patch_level(0);

	size_t nSize = initRequest.ByteSize();
	initRequest.SerializeToArray(&m_messageBodyBuffer[0], nSize);

	// build header for message
	m_header_request.nSequneceNumber = ++m_sequenceNumber;
	m_header_request.nLength = nSize;
	m_header_request.nCommandIdentifier = VS_INIT_REQUEST;

	// send request via stream. Payload is requestBuffer
	request(m_stream);
	memset(m_messageBodyBuffer, 0x00, sizeof(m_messageBodyBuffer));

	// receiving the response of VsCommunication and save it in m_messageBodyBuffer
	receive(m_stream);
	VMS::VisionSystem::InitResponseAck initRequestAck;
	// parsing message
	initRequestAck.ParseFromString(m_messageBodyBuffer);
	m_kLogger->debug("VisionSystem VersionId is: {}.{}.{}",
	                 initRequestAck.major_version(),
	                 initRequestAck.minor_version(),
	                 initRequestAck.patch_level());
}

uint64_t VmsMock::makeGetStatusRequest(uint32_t digitalIoCtrlToVs)
{
	m_stream = setUpStream(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);
	VMS::VisionSystem::GetStatusRequest getStatusRequest;
	getStatusRequest.set_digital_io_ctrl_to_vs(digitalIoCtrlToVs);

	size_t nSize = getStatusRequest.ByteSize();
	getStatusRequest.SerializeToArray(&m_messageBodyBuffer[0], nSize);

	// build header for message
	m_header_request.nSequneceNumber = ++m_sequenceNumber;
	m_header_request.nLength = nSize;
	m_header_request.nCommandIdentifier = VS_GET_STATUS;

	// send request via stream. Payload is requestBuffer
	request(m_stream);
	memset(m_messageBodyBuffer, 0x00, sizeof(m_messageBodyBuffer));
	// receiving the response of VsCommunication and save it in _messageBodyBuffer
	receive(m_stream);

	// Read message from VisionSystem
	VMS::VisionSystem::GetStatusResponseAck getStatusResponseAck;

	if (!getStatusResponseAck.ParseFromString(m_messageBodyBuffer))
	{
		throw std::runtime_error("getStatusResponseAck.ParseFromString failed");
	}

	if (getStatusResponseAck.ByteSize() == 0)
	{
		throw std::runtime_error("vmsMock Error: getStatusResponseAck.ByteSize == 0");
	}

	const size_t area = 0;
	uint64_t frameId = getStatusResponseAck.area_infos(area).frame_uid();
	m_kLogger->debug("frame_uid = {}", frameId);
	m_kLogger->debug("coord_x = {}", getStatusResponseAck.area_infos(area).product_infos(0).coord_x());
	return frameId;
}

void VmsMock::request(std::shared_ptr<TCPStream> stream)
{
	memset(m_requestBuffer, 0x00, sizeof(m_requestBuffer));
	// copy header and body in request and send it
	int header_size = sizeof(m_header_request);
	memcpy(m_requestBuffer, &m_header_request, header_size);
	memcpy(m_requestBuffer + header_size, m_messageBodyBuffer, sizeof(m_messageBodyBuffer) - header_size);
	stream->send(m_requestBuffer, sizeof(m_requestBuffer));
}

void VmsMock::receive(std::shared_ptr<TCPStream> stream)
{
	// zero buffer
	memset(m_responseBuffer, 0x00, sizeof(m_responseBuffer));
	memset(m_messageBodyBuffer, 0x00, sizeof(m_messageBodyBuffer));

	int nSocketSize = 0;
	int nBytesToRead = sizeof(m_header_response);
	// read header
	while (nBytesToRead > 0)
	{
		nSocketSize = stream->receive(m_responseBuffer, nBytesToRead, true);
		nBytesToRead -= nSocketSize;
	}

	// m_kLogger->debug("[RECEIVE] Headersize: {}", nSocketSize);
	memcpy(&m_header_response, &m_responseBuffer, sizeof(m_header_response));

	if (m_header_response.nStartIdentifier != VS_MESSAGE_STARTIDENTIFIER)
	{
		m_kLogger->debug("[RECEIVE] Unknown StartIdentifier {}", (int)m_header_response.nStartIdentifier);
		return;
	}
	m_sequenceNumber = m_header_response.nSequneceNumber;
	int nMessageLength = m_header_response.nLength;
	int nCommandIdentifier = m_header_response.nCommandIdentifier;
	int nStatus = m_header_response.nStatus;
	// m_kLogger->debug("[RECEIVE] StartIdentifier {}", m_header_response.nStartIdentifier);
	// m_kLogger->debug(" Seq: {}  Len: {}", m_header_response.nSequneceNumber, m_header_response.nLength);

	// read body message with length from header
	while (nMessageLength > 0)
	{
		nSocketSize = stream->receive(m_messageBodyBuffer, nMessageLength, true);
		nMessageLength -= nSocketSize;
	}
	if (nStatus != ACK)
	{
		checkStatus(nStatus, m_messageBodyBuffer);
		return;
	}
}

std::shared_ptr<TCPStream> VmsMock::setUpStream(const char* server, int port) const
{
	// try to connect with VsCommunication server
	std::shared_ptr<TCPConnector> connector = std::make_shared<TCPConnector>();
	std::shared_ptr<TCPStream> stream(connector->connect(server, port));
	if (!stream)
	{
		throw std::runtime_error(
		    "Could not create tcp stream. Is there any server up and running? If not start VsCommunication.");
	}
	return stream;
}

void VmsMock::checkStatus(const unsigned int nStatus, const char* cMessage) const
{
	switch (nStatus)
	{
		default:
		case NACK: {
			VMS::VisionSystem::ResponseNack responseNack;
			responseNack.ParseFromString(cMessage);

			switch (responseNack.error_code())
			{
				default:
				case 0x00000000: std::cout << "Error: none" << endl; break;
				case 0x00001001: std::cout << "Error: Parsing" << endl; break;
				case 0x00001002: std::cout << "Error: command id" << endl; break;
				case 0x00001003: std::cout << "Error: sequence no invalid" << endl; break;
				case 0x00001101: std::cout << "Error: from version" << endl; break;
				case 0x00001102: std::cout << "Error: no programm list" << endl; break;
				case 0x00001103: std::cout << "Error no camerasystem connected" << endl; break;
				case 0x00001104: std::cout << "Error Database entry missing" << endl; break;
				case 0x00001105: std::cout << "Error: unknown app id" << endl; break;
				case 0x00001106: std::cout << "Error: unknown robot type" << endl; break;
				case 0x00001107: std::cout << "Error format file not found" << endl; break;
				case 0xFFFFFFFF: std::cout << "Error: error" << endl; break;
			}
			std::cout << "[RECEIVE] Message: " << responseNack.DebugString().c_str() << "-----------------------"
			          << endl;

			break;
		}
		case BUSY: std::cout << "[RECEIVE] BUSY" << endl; break;
		case ERROR: std::cout << "[RECEIVE] ERROR" << endl; break;
	}
}