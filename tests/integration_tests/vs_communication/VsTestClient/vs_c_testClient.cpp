/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "vs_c_testClient.h"

#include <experimental/filesystem>
#include <iostream>

#include "Format.pb.h"
#include "GetStatus.pb.h"
#include "Init.pb.h"
#include "Nack.pb.h"
#include "gtest/gtest.h"
#include "projectPaths.h"

//! @ param database link
#define vsDatabaseLink "tcp://127.0.0.1:6379"

CommunicationTest::CommunicationTest() : _redis(sw::redis::Redis(vsDatabaseLink))
{
	std::experimental::filesystem::path path_to_vsCommunication =
	    utils::getProjectRootDir() / "build/bin/VsCommunication";
	if (!std::experimental::filesystem::exists(path_to_vsCommunication))
	{
		std::cerr << path_to_vsCommunication << " does not exist! Aborting ..." << std::endl;
		std::abort();
	}

	_process_child = bp::child(path_to_vsCommunication.string(), "");
	sleep(2);  // wait until VsCommunication is up
}

TEST_F(CommunicationTest, initialization)
{
	// create and initialize protobuf object
	VMS::VisionSystem::InitRequest initRequest;
	initRequest.set_major_version(1);
	initRequest.set_minor_version(0);
	initRequest.set_patch_level(0);

	size_t nSize = initRequest.ByteSize();
	initRequest.SerializeToArray(&_messageBodyBuffer[0], nSize);

	// build header for message
	_header_request.nSequneceNumber = ++_sequenceNumber;
	_header_request.nLength = nSize;
	_header_request.nCommandIdentifier = VS_INIT_REQUEST;

	std::shared_ptr<TCPStream> stream = setUpStream(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);

	std::cout << "[SEND | VS_INIT_REQUEST] Seq: " << _sequenceNumber << " Len: " << nSize
	          << " Message: " << initRequest.major_version() << "." << initRequest.minor_version() << "."
	          << initRequest.patch_level() << endl;
	// send request via stream. Payload is requestBuffer
	request(stream);

	// receiving the response of VsCommunication and save it in _messageBodyBuffer
	receive(stream);

	VMS::VisionSystem::InitResponseAck initRequestAck;

	// parsing message
	initRequestAck.ParseFromString(_messageBodyBuffer);
	std::cout << "[RECEIVE | VS_INIT_RESPONSE_ACK] Hardwaretype: " << initRequestAck.hardware_type()
	          << " Version:" << initRequestAck.major_version() << "." << initRequestAck.minor_version() << "."
	          << initRequestAck.patch_level() << endl;

	EXPECT_EQ(initRequestAck.major_version(), 1);
	EXPECT_EQ(initRequestAck.minor_version(), 0);
}

TEST_F(CommunicationTest, SendStatusToVMS)
{
	// put a pose object in database in order to enable VsCommunication
	// to read from it and send it back to this executable
	_redis.flushall();
	unordered_map<string, string> productMap{};
	ostringstream sProcessingProductKey, sProductCountKey;
	sProcessingProductKey << "processing:1" << PRODUCT_LIST;
	sProductCountKey << "processing:1:product:count";
	int max = 10;
	for (int t = 1; t < max; t++)
	{
		auto lProductCounter = _redis.incr(sProductCountKey.str());
		string sProductCounter = to_string(lProductCounter);
		string sKey = "processing:1";
		sKey.append(PRODUCT + sProductCounter);
		std::ostringstream sFrameIDKey;
		sFrameIDKey << "processing:1:frameCounter";
		_redis.set(sFrameIDKey.str(), "1");
		// publish product(s) to db
		productMap.insert(pair<string, string>({PRODUCT_UNIQUE_ID, to_string(t)}));
		productMap.insert(pair<string, string>({PRODUCT_ID, to_string(1)}));
		productMap.insert(pair<string, string>({PRODUCT_TIMESTAMP_SECONDS, to_string(1)}));
		productMap.insert(pair<string, string>({PRODUCT_TIMESTAMP_NANO, to_string(2)}));
		productMap.insert(
		    pair<string, string>({PRODUCT_PRIORITY, to_string(3)}));  // TODO: make value for each detected pose
		productMap.insert(
		    pair<string, string>({PRODUCT_ERROR, to_string(0)}));  // TODO: make value for each detected pose
		productMap.insert(pair<string, string>({PRODUCT_POSE_X, to_string(1)}));
		productMap.insert(pair<string, string>({PRODUCT_POSE_Y, to_string(2)}));
		productMap.insert(pair<string, string>({PRODUCT_POSE_Z, to_string(3)}));
		productMap.insert(pair<string, string>({PRODUCT_POSE_ANGLE_ROLL, to_string(4)}));
		productMap.insert(pair<string, string>({PRODUCT_POSE_ANGLE_PITCH, to_string(5)}));
		productMap.insert(pair<string, string>({PRODUCT_POSE_ANGLE_YAW, to_string(6)}));
		productMap.insert(pair<string, string>({PRODUCT_POSE_ROTATION_ENCODER, to_string(7)}));

		auto tx = _redis.transaction(true);

		tx.hmset(sKey, productMap.begin(), productMap.end())
		    .lpush(sProcessingProductKey.str(), sProductCounter)
		    .ltrim(sProcessingProductKey.str(), 0, max - 1)
		    .exec();
	}

	VMS::VisionSystem::GetStatusRequest getStatusRequest;
	// getStatusRequest.set_ea_output_image(VS_ACKNOWLEDGE_ERROR); //TODO FAKE VAL

	size_t nSize = getStatusRequest.ByteSize();
	getStatusRequest.SerializeToArray(&_messageBodyBuffer[0], nSize);

	_header_request.nSequneceNumber = ++_sequenceNumber;
	_header_request.nLength = nSize;
	_header_request.nCommandIdentifier = VS_GET_STATUS;

	std::shared_ptr<TCPStream> stream = setUpStream(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);

	// send request via stream. Payload is requestBuffer
	request(stream);

	// receiving the response of VsCommunication and save it in _messageBodyBuffer
	receive(stream);

	VMS::VisionSystem::GetStatusResponseAck getStatusResponseAck;

	// Read message
	std::cout << "[RECEIVE | VS_GET_STATUS_ACK] Message: -----------------------\n";
	std::cout << getStatusResponseAck.DebugString() << std::endl;
	getStatusResponseAck.ParseFromString(_messageBodyBuffer);

	for (int i = 0; i < getStatusResponseAck.area_infos_size(); i++)
	{
		const VMS::VisionSystem::AreaInfo& area = getStatusResponseAck.area_infos(i);

		std::cout << "Area number: " << area.area_number() << " ProductInfos: " << area.product_infos_size() << endl;
		for (int k = 0; k < area.product_infos_size(); k++)
		{
			const VMS::VisionSystem::ProductInfo& productInfo = area.product_infos(k);
			std::cout << "\e[1;32m"
			          << "Pose " << productInfo.coord_x() << "/" << productInfo.coord_y() << "/"
			          << productInfo.coord_z() << "/" << productInfo.angle_roll() << "/" << productInfo.angle_pitch()
			          << "/" << productInfo.angle_yaw() << endl;
		}
	}

	EXPECT_EQ(getStatusResponseAck.area_infos(0).product_infos(0).coord_x(),
	          10000);  // L&R expects mm/10 not
	                   // SI-Units --> factor is 10000
	                   // this factor is defined in visionSystemConfig.json
	EXPECT_EQ(getStatusResponseAck.area_infos(0).product_infos(0).coord_y(), 20000);
	EXPECT_EQ(getStatusResponseAck.area_infos(0).product_infos(0).coord_z(), 30000);
	EXPECT_EQ(getStatusResponseAck.area_infos(0).product_infos(0).angle_roll(),
	          int32_t(4 * 180.0 / 3.141592653589 * 10));
	EXPECT_EQ(getStatusResponseAck.area_infos(0).product_infos(0).angle_pitch(),
	          int32_t(5 * 180.0 / 3.141592653589 * 10));
	EXPECT_EQ(getStatusResponseAck.area_infos(0).product_infos(0).angle_yaw(),
	          int32_t(6 * 180.0 / 3.141592653589 * 10));
	EXPECT_EQ(getStatusResponseAck.area_infos(0).product_infos(0).encoder_value(), 7);
}

TEST_F(CommunicationTest, IsFormatLoaded)
{
	_redis.flushall();
	std::ostringstream sFormatLoaded;
	/*Currently there is only one format (unique id)*/
	sFormatLoaded << FORMAT << "1" << FORMAT_ACTIVE;
	_redis.set(sFormatLoaded.str(), "666");

	VMS::VisionSystem::FormatRequest formatRequest;
	formatRequest.set_id(666);
	size_t nSize = formatRequest.ByteSize();
	formatRequest.SerializeToArray(&_messageBodyBuffer[0], nSize);

	_header_request.nSequneceNumber = ++_sequenceNumber;
	_header_request.nLength = nSize;
	_header_request.nCommandIdentifier = VS_FORMAT_LOADED;

	std::shared_ptr<TCPStream> stream = setUpStream(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);
	// send request via stream. Payload is requestBuffer
	request(stream);
	// receiving the response of VsCommunication and save it in _messageBodyBuffer
	receive(stream);

	// parsing message
	VMS::VisionSystem::FormatLoadedResponse format;
	format.ParseFromString(_messageBodyBuffer);
	std::cout << "[RECEIVE | VS_FORMAT_LOADED] Message: -----------------------\n";
	std::cout << format.DebugString() << endl;

	EXPECT_EQ(format.id(), 666);
}

void CommunicationTest::request(const std::shared_ptr<TCPStream>& stream)
{
	memset(_requestBuffer, 0x00, sizeof(_requestBuffer));
	// copy header and body in request and send it
	int header_size = sizeof(_header_request);
	memcpy(_requestBuffer, &_header_request, header_size);
	memcpy(_requestBuffer + header_size, _messageBodyBuffer, sizeof(_messageBodyBuffer) - header_size);
	stream->send(_requestBuffer, sizeof(_requestBuffer));
}

void CommunicationTest::receive(const std::shared_ptr<TCPStream>& stream)
{
	// zero buffer
	memset(_responseBuffer, 0x00, sizeof(_responseBuffer));
	memset(_messageBodyBuffer, 0x00, sizeof(_messageBodyBuffer));

	int nSocketSize = 0;
	int nBytesToRead = sizeof(_header_response);
	// read header
	while (nBytesToRead > 0)
	{
		nSocketSize = stream->receive(_responseBuffer, nBytesToRead, true);
		nBytesToRead -= nSocketSize;
	}

	std::cout << "[RECEIVE] Headersize: " << nSocketSize << endl;
	memcpy(&_header_response, &_responseBuffer, sizeof(_header_response));

	if (_header_response.nStartIdentifier != VS_MESSAGE_STARTIDENTIFIER)
	{
		std::cout << "[RECEIVE] Unknown StartIdentifier " << (int)_header_response.nStartIdentifier << endl;
		return;
	}
	_sequenceNumber = _header_response.nSequneceNumber;
	int nMessageLength = _header_response.nLength;
	int nCommandIdentifier = _header_response.nCommandIdentifier;
	int nStatus = _header_response.nStatus;
	std::cout << "[RECEIVE] StartIdentifier " << _header_response.nStartIdentifier
	          << " Seq: " << _header_response.nSequneceNumber << " Len: " << _header_response.nLength << endl;

	// read body message with length from header
	while (nMessageLength > 0)
	{
		nSocketSize = stream->receive(_messageBodyBuffer, nMessageLength, true);
		nMessageLength -= nSocketSize;
	}
	if (nStatus != ACK)
	{
		checkStatus(nStatus, _messageBodyBuffer);
		return;
	}
}

std::shared_ptr<TCPStream> CommunicationTest::setUpStream(const char* server, int port) const
{
	// try to connect with VsCommunication server
	std::shared_ptr<TCPConnector> connector = std::make_shared<TCPConnector>();
	std::shared_ptr<TCPStream> stream(connector->connect(server, port));
	if (!stream)
	{
		std::cout << "No stream created --> connection()" << std::endl;
		std::cout << "Is there any server up and running? If not start VsCommunication." << std::endl;
	}
	return stream;
}

void CommunicationTest::checkStatus(const int nStatus, const char* cMessage) const
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

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}