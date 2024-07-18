/*
 * Created on Wed Oct 24 2018
 *
 * Copyright (c) 2018 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 */

#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <string>

// proto files
#include "Format.pb.h"
#include "GetStatus.pb.h"
#include "Init.pb.h"
#include "StartProduction.pb.h"
//#include "StopProduction.pb.h"
//#include "Reset.pb.h"
#include "AppData.pb.h"
#include "LiveImage.pb.h"
#include "Marker.pb.h"
#include "Nack.pb.h"
#include "Program.pb.h"
#include "RoboterSettings.pb.h"
#include "projectPaths.h"
#include "vs.h"
#include "vs_c_tcpConnector.h"
#include "vs_messaging.h"

#define CVUI_IMPLEMENTATION
#include "../libs/cvui.h"

using namespace std;
using namespace VMS::VisionSystem;

std::shared_ptr<db::DbFacade> m_dbFacade =
    std::make_shared<db::DbFacade>(utils::getProjectRootDir() / "config/visionSystemConfig.json");

vs_message_header_request send_request;
vs_message_header_response send_response;

int nFirstProgram = 2, nSecondProgram = 4, nAppData = 1, nMarker = 1, nCalibration = 1;

static void checkStatus(int nStatus, const char* cMessage)
{
	switch (nStatus)
	{
		default:
		case NACK: {
			ResponseNack responseNack;
			responseNack.ParseFromString(cMessage);

			switch (responseNack.error_code())
			{
				default:
				case 0x00000000: cout << "Error: none" << endl; break;
				case 0x00001001: cout << "Error: Parsing" << endl; break;
				case 0x00001002: cout << "Error: command id" << endl; break;
				case 0x00001003: cout << "Error: sequence no invalid" << endl; break;
				case 0x00001101: cout << "Error: from version" << endl; break;
				case 0x00001102: cout << "Error: no programm list" << endl; break;
				case 0x00001103: cout << "Error no camerasystem connected" << endl; break;
				case 0x00001104: cout << "Error Database entry missing" << endl; break;
				case 0x00001105: cout << "Error: unknown app id" << endl; break;
				case 0x00001106: cout << "Error: unknown robot type" << endl; break;
				case 0x00001107: cout << "Error format file not found" << endl; break;
				case 0xFFFFFFFF: cout << "Error: error" << endl; break;
			}
			cout << "[RECEIVE] Message: " << responseNack.DebugString().c_str() << "-----------------------" << endl;

			break;
		}
		case BUSY: cout << "[RECEIVE] BUSY" << endl; break;
		case ERROR: cout << "[RECEIVE] ERROR" << endl; break;
	}
}

int main(int argc, char** argv)
{
	int c;
	const char* short_opt = "h";
	struct option long_opt[] = {{"help", no_argument, NULL, 'h'}, {NULL, 0, NULL, 0}};

	cv::Mat frame = cv::Mat(300, 750, CV_8UC3);

	static int nSequenceNumber = 0;
	char requestBuffer[VS_MESSAGE_BUFFER_SIZE];
	char responseBuffer[VS_MESSAGE_BUFFER_SIZE];
	char messageBuffer[VS_MESSAGE_BUFFER_SIZE];

	vector<int> vProgramList;
	int nProgramID = 0;
	bool bInitProgram = false, bCameraLiveImage = false, bVSLiveImage = false;

	// Create Window
	cvui::init(argv[0]);

	while ((c = getopt_long(argc, argv, short_opt, long_opt, NULL)) != -1)
	{
		switch (c)
		{
			case 'h':
			case ':':
			case '?':
			default:
				cout << "Usage: " << argv[0] << " [OPTIONS]" << endl;
				cout << "\t-h --help\tprint this help and exit" << endl << endl;

				return (0);
		}
	}

	TCPConnector* connector = new TCPConnector();
	TCPStream* stream;
	stream = connector->connect(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);

	// zero buffer
	memset(requestBuffer, 0x00, sizeof(requestBuffer));
	memset(responseBuffer, 0x00, sizeof(responseBuffer));
	memset(messageBuffer, 0x00, sizeof(messageBuffer));
	// init
	if (stream)
	{
		InitRequest initRequest;

		initRequest.set_major_version(1);
		initRequest.set_minor_version(0);
		initRequest.set_patch_level(0);

		size_t nSize = initRequest.ByteSize();
		initRequest.SerializeToArray(&messageBuffer[0], nSize);

		send_request.nSequneceNumber = ++nSequenceNumber;
		send_request.nLength = nSize;
		send_request.nCommandIdentifier = VS_INIT_REQUEST;

		int nSendMessageSize = sizeof(send_request);

		memcpy(requestBuffer, &send_request, nSendMessageSize);
		memcpy(requestBuffer + nSendMessageSize, messageBuffer, sizeof(messageBuffer));

		cout << "[SEND | VS_INIT_REQUEST] Seq: " << nSequenceNumber << " Len: " << nSize
		     << " Message: " << initRequest.major_version() << "." << initRequest.minor_version() << "."
		     << initRequest.patch_level() << endl;
		// cout << "--->" << initRequest.DebugString() << endl;
		stream->send(requestBuffer, sizeof(requestBuffer));

		int nSocketSize = 0;
		int nBytesToRead = sizeof(send_response);

		// read header
		while (nBytesToRead > 0)
		{
			nSocketSize = stream->receive(responseBuffer, nBytesToRead, true);
			nBytesToRead -= nSocketSize;
		}

		cout << "[RECEIVE] Headersize: " << nSocketSize << endl;

		memcpy(&send_response, &responseBuffer, sizeof(send_response));
		if (send_response.nStartIdentifier != VS_MESSAGE_STARTIDENTIFIER)
		{
			cout << "[RECEIVE] Unknown StartIdentifier " << (int)send_response.nStartIdentifier << endl;
			delete stream;
			exit(-1);
		}
		int nSequenceNumber = send_response.nSequneceNumber;
		int nMessageLength = send_response.nLength;
		int nCommandIdentifier = send_response.nCommandIdentifier;
		int nStatus = send_response.nStatus;
		cout << "[RECEIVE] StartIdentifier " << send_response.nStartIdentifier
		     << " Seq: " << send_response.nSequneceNumber << " Len: " << send_response.nLength << endl;

		// read body message with length from header
		while (nMessageLength > 0)
		{
			nSocketSize = stream->receive(messageBuffer, nMessageLength, true);
			nMessageLength -= nSocketSize;
		}

		if (nStatus != ACK)
		{
			checkStatus(nStatus, messageBuffer);

			delete stream;
			exit(-1);
		}
		else
		{
			InitResponseAck initRequestAck;

			// Read message
			initRequestAck.ParseFromString(messageBuffer);
			cout << "[RECEIVE | VS_INIT_RESPONSE_ACK] Hardwaretype: " << initRequestAck.hardware_type()
			     << " Version:" << initRequestAck.major_version() << "." << initRequestAck.minor_version() << "."
			     << initRequestAck.patch_level() << endl;
		}

		delete stream;
	}  // end of initialisation
	else
	{
		cout << "No stream created --> connection()" << endl;
		exit(1);
	}

	cout << "------Initialisation successed------" << endl;

	while (1)
	{
		frame = cv::Scalar(49, 52, 49);

		if (cvui::button(frame, 10, 50, "GetAppMessage"))
		{
			stream = connector->connect(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);
			if (stream)
			{
				memset(requestBuffer, 0x00, sizeof(requestBuffer));

				AppMessageRequest appRequest;

				appRequest.set_code(100);

				size_t nSize = appRequest.ByteSize();
				appRequest.SerializeToArray(&messageBuffer[0], nSize);

				send_request.nSequneceNumber = ++nSequenceNumber;
				send_request.nLength = nSize;
				send_request.nCommandIdentifier = VS_APP_MESSAGE_GET;

				int nSendMessageSize = sizeof(send_request);

				memcpy(requestBuffer, &send_request, nSendMessageSize);
				memcpy(requestBuffer + nSendMessageSize, messageBuffer, sizeof(messageBuffer));
				stream->send(requestBuffer, sizeof(requestBuffer));
				cout << "[SEND | VS_APP_MESSAGE_GET] Seq: " << nSequenceNumber << " Len: " << nSize << " Message: []"
				     << endl;

				// zero buffer
				memset(responseBuffer, 0x00, sizeof(responseBuffer));
				// receive message
				int nSocketSize = 0;
				int nBytesToRead = sizeof(send_response);

				// zero buffer
				memset(responseBuffer, 0x00, sizeof(responseBuffer));
				// read header
				while (nBytesToRead > 0)
				{
					nSocketSize = stream->receive(responseBuffer, nBytesToRead, true);
					nBytesToRead -= nSocketSize;
				}

				cout << "[RECEIVE] Headersize: " << nSocketSize << endl;

				memcpy(&send_response, &responseBuffer, sizeof(send_response));
				if (send_response.nStartIdentifier != VS_MESSAGE_STARTIDENTIFIER)
				{
					cout << "[RECEIVE] Unknown StartIdentifier " << (int)send_response.nStartIdentifier << endl;
					delete stream;
					exit(-1);
				}
				int nSequenceNumber = send_response.nSequneceNumber;
				int nMessageLength = send_response.nLength;
				int nCommandIdentifier = send_response.nCommandIdentifier;
				int nStatus = send_response.nStatus;
				cout << "[RECEIVE] StartIdentifier " << send_response.nStartIdentifier
				     << " Seq: " << send_response.nSequneceNumber << " Len: " << send_response.nLength << endl;

				// read body message with length from header
				while (nMessageLength > 0)
				{
					nSocketSize = stream->receive(messageBuffer, nMessageLength, true);
					nMessageLength -= nSocketSize;
				}
				if (send_response.nStatus != ACK)
				{
					checkStatus(send_response.nStatus, messageBuffer);
				}
				else
				{
					AppMessageResponse appResponse;

					// Read message
					appResponse.ParseFromString(messageBuffer);
					cout << "[RECEIVE] VS_APP_MESSAGE_GET ACK Message: -----------------------"
					     << appResponse.DebugString() << endl;
				}
				delete (stream);
			}
		}

		if (cvui::button(frame, 10, 130, "Start"))
		{
			stream = connector->connect(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);
			if (stream)
			{
				memset(requestBuffer, 0x00, sizeof(requestBuffer));

				StartProductionRequest startProductionRequest;

				startProductionRequest.set_program_id((uint32_t)INT32_MAX);

				size_t nSize = startProductionRequest.ByteSize();
				startProductionRequest.SerializeToArray(&messageBuffer[0], nSize);

				send_request.nSequneceNumber = ++nSequenceNumber;
				send_request.nLength = nSize;
				send_request.nCommandIdentifier = VS_START_PRODUCTION;

				int nSendMessageSize = sizeof(send_request);

				memcpy(requestBuffer, &send_request, nSendMessageSize);
				memcpy(requestBuffer + nSendMessageSize, messageBuffer, sizeof(messageBuffer));
				stream->send(requestBuffer, sizeof(requestBuffer));
				cout << "[SEND | VS_START_PRODUCTION] Seq: " << nSequenceNumber << " Len: " << nSize
				     << " Message: " << startProductionRequest.program_id() << endl;

				// zero buffer
				memset(responseBuffer, 0x00, sizeof(responseBuffer));
				// receive message
				int nSocketSize = 0;
				int nBytesToRead = sizeof(send_response);

				// zero buffer
				memset(responseBuffer, 0x00, sizeof(responseBuffer));
				// read header
				while (nBytesToRead > 0)
				{
					nSocketSize = stream->receive(responseBuffer, nBytesToRead, true);
					nBytesToRead -= nSocketSize;
				}

				cout << "[RECEIVE] Headersize: " << nSocketSize << endl;

				memcpy(&send_response, &responseBuffer, sizeof(send_response));
				if (send_response.nStartIdentifier != VS_MESSAGE_STARTIDENTIFIER)
				{
					cout << "[RECEIVE] Unknown StartIdentifier " << (int)send_response.nStartIdentifier << endl;
					delete stream;
					exit(-1);
				}
				int nSequenceNumber = send_response.nSequneceNumber;
				int nMessageLength = send_response.nLength;
				int nCommandIdentifier = send_response.nCommandIdentifier;
				int nStatus = send_response.nStatus;
				cout << "[RECEIVE] StartIdentifier " << send_response.nStartIdentifier
				     << " Seq: " << send_response.nSequneceNumber << " Len: " << send_response.nLength << endl;

				// read body message with length from header
				while (nMessageLength > 0)
				{
					nSocketSize = stream->receive(messageBuffer, nMessageLength, true);
					nMessageLength -= nSocketSize;
				}
				if (send_response.nStatus != ACK)
				{
					checkStatus(send_response.nStatus, messageBuffer);
				}
				else
				{
					cout << "[RECEIVE] START PRODUCTION_ACK" << endl;
				}
				delete (stream);
			}
		}
		if (cvui::button(frame, 130, 130, "Send format"))
		{
			stream = connector->connect(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);
			if (stream)
			{
				memset(requestBuffer, 0x00, sizeof(requestBuffer));

				FormatRequest formatRequest;
				formatRequest.set_id(666);
				size_t nSize = formatRequest.ByteSize();
				formatRequest.SerializeToArray(&messageBuffer[0], nSize);

				send_request.nSequneceNumber = ++nSequenceNumber;
				send_request.nLength = nSize;
				send_request.nCommandIdentifier = VS_FORMAT;

				int nSendMessageSize = sizeof(send_request);

				memcpy(requestBuffer, &send_request, nSendMessageSize);
				memcpy(requestBuffer + nSendMessageSize, messageBuffer, sizeof(messageBuffer));
				stream->send(requestBuffer, sizeof(requestBuffer));
				cout << "[SEND | VS_FORMAT] Seq: " << nSequenceNumber << " Len: " << nSize << " Message: []" << endl;

				delete (stream);
			}
		}

		if (cvui::button(frame, 130, 170, "format available"))
		{
			stream = connector->connect(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);
			if (stream)
			{
				memset(requestBuffer, 0x00, sizeof(requestBuffer));

				FormatRequest formatRequest;
				formatRequest.set_id(666);
				size_t nSize = formatRequest.ByteSize();
				formatRequest.SerializeToArray(&messageBuffer[0], nSize);

				send_request.nSequneceNumber = ++nSequenceNumber;
				send_request.nLength = nSize;
				send_request.nCommandIdentifier = VS_FORMAT_AVAILABLE;

				int nSendMessageSize = sizeof(send_request);

				memcpy(requestBuffer, &send_request, nSendMessageSize);
				memcpy(requestBuffer + nSendMessageSize, messageBuffer, sizeof(messageBuffer));
				stream->send(requestBuffer, sizeof(requestBuffer));
				cout << "[SEND | VS_FORMAT_AVAILABLE] Seq: " << nSequenceNumber << " Len: " << nSize << " Message: []"
				     << endl;

				// zero buffer
				memset(responseBuffer, 0x00, sizeof(responseBuffer));
				// receive message
				int nSocketSize = 0;
				int nBytesToRead = sizeof(send_response);

				// zero buffer
				memset(responseBuffer, 0x00, sizeof(responseBuffer));
				// read header
				while (nBytesToRead > 0)
				{
					nSocketSize = stream->receive(responseBuffer, nBytesToRead, true);
					nBytesToRead -= nSocketSize;
				}

				cout << "[RECEIVE] Headersize: " << nSocketSize << endl;

				memcpy(&send_response, &responseBuffer, sizeof(send_response));
				if (send_response.nStartIdentifier != VS_MESSAGE_STARTIDENTIFIER)
				{
					cout << "[RECEIVE] Unknown StartIdentifier " << (int)send_response.nStartIdentifier << endl;
					delete stream;
					exit(-1);
				}
				int nSequenceNumber = send_response.nSequneceNumber;
				int nMessageLength = send_response.nLength;
				int nCommandIdentifier = send_response.nCommandIdentifier;
				int nStatus = send_response.nStatus;
				cout << "[RECEIVE] StartIdentifier " << send_response.nStartIdentifier
				     << " Seq: " << send_response.nSequneceNumber << " Len: " << send_response.nLength << endl;

				// read body message with length from header
				while (nMessageLength > 0)
				{
					nSocketSize = stream->receive(messageBuffer, nMessageLength, true);
					nMessageLength -= nSocketSize;
				}
				if (send_response.nStatus != ACK)
				{
					checkStatus(send_response.nStatus, messageBuffer);
				}
				else
				{
					FormatAvailableResponse format;
					format.ParseFromString(messageBuffer);
					cout << "[RECEIVE | VS_FORMAT_AVAILABLE] Message: -----------------------" << format.DebugString()
					     << endl;
				}
				delete (stream);
			}
		}

		if (cvui::button(frame, 130, 210, "format loaded"))
		{
			stream = connector->connect(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);
			if (stream)
			{
				memset(requestBuffer, 0x00, sizeof(requestBuffer));

				FormatRequest formatRequest;
				formatRequest.set_id(666);
				size_t nSize = formatRequest.ByteSize();
				formatRequest.SerializeToArray(&messageBuffer[0], nSize);

				send_request.nSequneceNumber = ++nSequenceNumber;
				send_request.nLength = nSize;
				send_request.nCommandIdentifier = VS_FORMAT_LOADED;

				int nSendMessageSize = sizeof(send_request);

				memcpy(requestBuffer, &send_request, nSendMessageSize);
				memcpy(requestBuffer + nSendMessageSize, messageBuffer, sizeof(messageBuffer));
				stream->send(requestBuffer, sizeof(requestBuffer));
				cout << "[SEND | VS_FORMAT_LOADED] Seq: " << nSequenceNumber << " Len: " << nSize << " Message: []"
				     << endl;

				// zero buffer
				memset(responseBuffer, 0x00, sizeof(responseBuffer));
				// receive message
				int nSocketSize = 0;
				int nBytesToRead = sizeof(send_response);

				// zero buffer
				memset(responseBuffer, 0x00, sizeof(responseBuffer));
				// read header
				while (nBytesToRead > 0)
				{
					nSocketSize = stream->receive(responseBuffer, nBytesToRead, true);
					nBytesToRead -= nSocketSize;
				}

				cout << "[RECEIVE] Headersize: " << nSocketSize << endl;

				memcpy(&send_response, &responseBuffer, sizeof(send_response));
				if (send_response.nStartIdentifier != VS_MESSAGE_STARTIDENTIFIER)
				{
					cout << "[RECEIVE] Unknown StartIdentifier " << (int)send_response.nStartIdentifier << endl;
					delete stream;
					exit(-1);
				}
				int nSequenceNumber = send_response.nSequneceNumber;
				int nMessageLength = send_response.nLength;
				int nCommandIdentifier = send_response.nCommandIdentifier;
				int nStatus = send_response.nStatus;
				cout << "[RECEIVE] StartIdentifier " << send_response.nStartIdentifier
				     << " Seq: " << send_response.nSequneceNumber << " Len: " << send_response.nLength << endl;

				// read body message with length from header
				while (nMessageLength > 0)
				{
					nSocketSize = stream->receive(messageBuffer, nMessageLength, true);
					nMessageLength -= nSocketSize;
				}
				if (send_response.nStatus != ACK)
				{
					checkStatus(send_response.nStatus, messageBuffer);
				}
				else
				{
					FormatLoadedResponse format;
					format.ParseFromString(messageBuffer);
					cout << "[RECEIVE | VS_FORMAT_LOADED] Message: -----------------------" << format.DebugString()
					     << endl;
				}
				delete (stream);
			}
		}

		if (cvui::button(frame, 10, 170, "Get Status"))
		{
			double tick = (double)cv::getTickCount();
			stream = connector->connect(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);
			if (stream)
			{
				cout << "connect = " << ((double)cv::getTickCount() - tick) / cv::getTickFrequency() * 1000 << " ms"
				     << endl;
				memset(requestBuffer, 0x00, sizeof(requestBuffer));

				GetStatusRequest getStatusRequest;

				size_t nSize = getStatusRequest.ByteSize();
				getStatusRequest.SerializeToArray(&messageBuffer[0], nSize);

				send_request.nSequneceNumber = ++nSequenceNumber;
				send_request.nLength = nSize;
				send_request.nCommandIdentifier = VS_GET_STATUS;

				int nSendMessageSize = sizeof(send_request);

				memcpy(requestBuffer, &send_request, nSendMessageSize);
				memcpy(requestBuffer + nSendMessageSize, messageBuffer, sizeof(messageBuffer));

				stream->send(requestBuffer, sizeof(requestBuffer));
				cout << "send = " << ((double)cv::getTickCount() - tick) / cv::getTickFrequency() * 1000 << " ms"
				     << endl;

				cout << "[SEND | VS_GET_STATUS] Seq: " << nSequenceNumber << " Len: " << nSize
				     << " EA Output: " << getStatusRequest.digital_io_ctrl_to_vs() << endl;

				unordered_map<string, string> productMap{};
				ostringstream sProcessingProductKey, sProductCountKey;
				sProcessingProductKey << "processing:1" << PRODUCT_LIST;
				sProductCountKey << "processing:1:product:count";
				int max = 10;
				std::vector<VSPose> posesInRobotCoordinates;
				std::vector<int> classIds;
				int32_t rotEncoderValue = 0;
				uint64_t frameId = 0;
				uint32_t areaNumber = 0;
				for (int t = 0; t < max; t++)
				{
					VSPose pose;
					pose.x = 666.0F;
					pose.y = 777.0F;
					pose.z = 888.0F;
					pose.roll = 1.0F;
					pose.pitch = 2.0F;
					pose.yaw = 3.0F;
					posesInRobotCoordinates.pushback(pose);
					classIds.push_back(0);
				}
				m_dbFacade->serializeProductInfosToDB(posesInRobotCoordinates,
				                                      classIds,
				                                      rotEncoderValue,
				                                      frameId,
				                                      areaNumber,
				                                      db::ObjectType::product);
				double tick2 = (double)cv::getTickCount();
				// zero buffer
				memset(responseBuffer, 0x00, sizeof(responseBuffer));

				int nSocketSize = 0;
				int nBytesToRead = sizeof(send_response);

				// zero buffer
				memset(responseBuffer, 0x00, sizeof(responseBuffer));
				// read header
				while (nBytesToRead > 0)
				{
					nSocketSize = stream->receive(responseBuffer, nBytesToRead, true);
					nBytesToRead -= nSocketSize;
				}

				cout << "[RECEIVE] Headersize: " << nSocketSize << endl;

				memcpy(&send_response, &responseBuffer, sizeof(send_response));
				if (send_response.nStartIdentifier != VS_MESSAGE_STARTIDENTIFIER)
				{
					cout << "[RECEIVE] Unknown StartIdentifier " << (int)send_response.nStartIdentifier << endl;
					delete stream;
					exit(-1);
				}
				int nSequenceNumber = send_response.nSequneceNumber;
				int nMessageLength = send_response.nLength;
				int nCommandIdentifier = send_response.nCommandIdentifier;
				int nStatus = send_response.nStatus;
				cout << "[RECEIVE] StartIdentifier " << send_response.nStartIdentifier
				     << " Seq: " << send_response.nSequneceNumber << " Len: " << send_response.nLength << endl;

				// read body message with length from header
				while (nMessageLength > 0)
				{
					nSocketSize = stream->receive(messageBuffer, nMessageLength, true);
					nMessageLength -= nSocketSize;
				}

				// receive message
				if (send_response.nStatus != ACK)
				{
					checkStatus(send_response.nStatus, messageBuffer);
				}
				else
				{
					cout << "receive = " << ((double)cv::getTickCount() - tick2) / cv::getTickFrequency() * 1000
					     << " ms" << endl;
					cout << "[RECEIVE] VS_GET_STATUS_ACK" << endl;

					GetStatusResponseAck getStatusResponseAck;

					cout << "[RECEIVE] Seq: " << send_response.nSequneceNumber << " Len: " << nMessageLength << endl;

					// Read message
					getStatusResponseAck.ParseFromString(messageBuffer);
					// cout << "[RECEIVE | VS_GET_STATUS_ACK] Message: -----------------------" <<
					// getStatusResponseAck.DebugString() << endl;

					for (int i = 0; i < getStatusResponseAck.area_infos_size(); i++)
					{
						const AreaInfo& area = getStatusResponseAck.area_infos(i);

						cout << "Area number: " << area.area_number() << " ProductInfos: " << area.product_infos_size()
						     << endl;
						for (int k = 0; k < area.product_infos_size(); k++)
						{
							const ProductInfo& productInfo = area.product_infos(k);
							cout << "\e[1;32m "
							     << " Pose " << productInfo.coord_x() << "/" << productInfo.coord_y() << "/"
							     << productInfo.coord_z() << "/" << productInfo.angle_roll() << "/"
							     << productInfo.angle_pitch() << "/" << productInfo.angle_yaw() << endl;
						}
					}
					delete (stream);
				}
			}
		}

		////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////

		cvui::text(frame, 300, 10, "To/From Backend", 0.4, 0x000000);

		if (cvui::button(frame, 450, 30, "Get camera systems"))
		{
			stream = connector->connect(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);
			if (stream)
			{
				memset(requestBuffer, 0x00, sizeof(requestBuffer));
				send_request.nSequneceNumber = ++nSequenceNumber;
				send_request.nLength = 0;
				send_request.nCommandIdentifier = VS_CAMERA_SYSTEMS_GET;

				int nSendMessageSize = sizeof(send_request);

				memcpy(requestBuffer, &send_request, nSendMessageSize);
				memcpy(requestBuffer + nSendMessageSize, messageBuffer, sizeof(messageBuffer));

				stream->send(requestBuffer, sizeof(requestBuffer));
				cout << "[SEND | VS_GET_CAMERA_SYSTEMS] Seq: " << nSequenceNumber << endl;

				// zero buffer
				memset(responseBuffer, 0x00, sizeof(responseBuffer));
				// receive message
				int nSocketSize = 0;
				int nBytesToRead = sizeof(send_response);

				// zero buffer
				memset(responseBuffer, 0x00, sizeof(responseBuffer));
				// read header
				while (nBytesToRead > 0)
				{
					nSocketSize = stream->receive(responseBuffer, nBytesToRead, true);
					nBytesToRead -= nSocketSize;
				}

				cout << "[RECEIVE] Headersize: " << nSocketSize << endl;

				memcpy(&send_response, &responseBuffer, sizeof(send_response));
				if (send_response.nStartIdentifier != VS_MESSAGE_STARTIDENTIFIER)
				{
					cout << "[RECEIVE] Unknown StartIdentifier " << (int)send_response.nStartIdentifier << endl;
					delete stream;
					exit(-1);
				}
				int nSequenceNumber = send_response.nSequneceNumber;
				int nMessageLength = send_response.nLength;
				int nCommandIdentifier = send_response.nCommandIdentifier;
				int nStatus = send_response.nStatus;
				cout << "[RECEIVE] StartIdentifier " << send_response.nStartIdentifier
				     << " Seq: " << send_response.nSequneceNumber << " Len: " << send_response.nLength << endl;

				// read body message with length from header
				while (nMessageLength > 0)
				{
					nSocketSize = stream->receive(messageBuffer, nMessageLength, true);
					nMessageLength -= nSocketSize;
				}
				if (send_response.nStatus != ACK)
				{
					checkStatus(send_response.nStatus, messageBuffer);
				}
				else
				{
					cout << "[RECEIVE] START VS_GET_CAMERA_SYSTEMS" << endl;
					Robot robot;
					// read message
					robot.ParseFromString(messageBuffer);
					cout << "\tRoboter: \"" << robot.robot_name() << "\" " << robot.robot_id() << endl;
					cout << "\tVision System(e): " << robot.vision_system_size() << endl;
					for (int vs = 0; vs < robot.vision_system_size(); vs++)
					{
						VisionSystem vision_system = robot.vision_system(vs);

						cout << "\t\tVs - ID: " << vision_system.vision_system_id() << endl;
						cout << "\t\tVs - Name: " << vision_system.vision_system_name() << endl;
						cout << "\t\tVs - Port: " << vision_system.vision_system_port() << endl;

						for (int cn = 0; cn < vision_system.camera_size(); cn++)
						{
							Camera camera = vision_system.camera(cn);

							cout << "\t\t\tC - ID: " << camera.camera_id() << endl;
							cout << "\t\t\tC - Name: " << camera.camera_name() << endl;
							cout << "\t\t\tC - Port: " << camera.camera_port() << endl;
						}
					}
				}
				delete (stream);
			}
		}

		if (cvui::button(frame, 300, 30, "Get Program list"))
		{
			stream = connector->connect(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);
			if (stream)
			{
				memset(requestBuffer, 0x00, sizeof(requestBuffer));
				send_request.nSequneceNumber = ++nSequenceNumber;
				send_request.nLength = 0;
				send_request.nCommandIdentifier = VS_PROGRAM_AVAILABLE;

				int nSendMessageSize = sizeof(send_request);

				memcpy(requestBuffer, &send_request, nSendMessageSize);
				memcpy(requestBuffer + nSendMessageSize, messageBuffer, sizeof(messageBuffer));

				stream->send(requestBuffer, sizeof(requestBuffer));
				cout << "[SEND | VS_PROGRAM_AVAILABLE] Seq: " << nSequenceNumber << endl;

				// zero buffer
				memset(responseBuffer, 0x00, sizeof(responseBuffer));
				// receive message
				int nSocketSize = 0;
				int nBytesToRead = sizeof(send_response);

				// zero buffer
				memset(responseBuffer, 0x00, sizeof(responseBuffer));
				// read header
				while (nBytesToRead > 0)
				{
					nSocketSize = stream->receive(responseBuffer, nBytesToRead, true);
					nBytesToRead -= nSocketSize;
				}

				cout << "[RECEIVE] Headersize: " << nSocketSize << endl;

				memcpy(&send_response, &responseBuffer, sizeof(send_response));
				if (send_response.nStartIdentifier != VS_MESSAGE_STARTIDENTIFIER)
				{
					cout << "[RECEIVE] Unknown StartIdentifier " << (int)send_response.nStartIdentifier << endl;
					delete stream;
					exit(-1);
				}
				int nSequenceNumber = send_response.nSequneceNumber;
				int nMessageLength = send_response.nLength;
				int nCommandIdentifier = send_response.nCommandIdentifier;
				int nStatus = send_response.nStatus;
				cout << "[RECEIVE] StartIdentifier " << send_response.nStartIdentifier
				     << " Seq: " << send_response.nSequneceNumber << " Len: " << send_response.nLength << endl;

				// read body message with length from header
				while (nMessageLength > 0)
				{
					nSocketSize = stream->receive(messageBuffer, nMessageLength, true);
					nMessageLength -= nSocketSize;
				}
				if (send_response.nStatus != ACK)
				{
					checkStatus(send_response.nStatus, messageBuffer);
				}
				else
				{
					cout << "[RECEIVE] START VS_PROGRAM_AVAILABLE" << endl;
					ProgramAvailableResponse programAvailableResponse;
					// Read message
					programAvailableResponse.ParseFromString(messageBuffer);
					cout << "ProgramInstalled: " << programAvailableResponse.program_list_size() << endl;

					for (int p = 0; p < programAvailableResponse.program_list_size(); p++)
					{
						cout << p << ": " << programAvailableResponse.program_list(p) << endl;
						vProgramList.push_back(programAvailableResponse.program_list(p));
					}
				}
				bInitProgram = true;
				delete (stream);
			}
		}
		if (bInitProgram)
		{
			cvui::counter(frame, 480, 70, &nFirstProgram);
			cvui::counter(frame, 580, 70, &nSecondProgram);

			if (cvui::button(frame, 300, 70, "Send vision-program"))
			{
				stream = connector->connect(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);
				if (stream)
				{
					ProgramRequest programRequest;
					ostringstream sProgramName;

					nProgramID++;
					programRequest.set_id(nProgramID);

					sProgramName << "TestProgram_" << nProgramID;
					programRequest.set_name(sProgramName.str());
					// toDo
					// creation_date
					// last_Update
					programRequest.add_program(vProgramList.at(nFirstProgram));
					programRequest.add_program(vProgramList.at(nSecondProgram));
					programRequest.set_vision_system_id(1);

					size_t nSize = programRequest.ByteSize();
					programRequest.SerializeToArray(&messageBuffer[0], nSize);

					send_request.nSequneceNumber = ++nSequenceNumber;
					send_request.nLength = nSize;
					send_request.nCommandIdentifier = VS_PROGRAM;

					int nSendMessageSize = sizeof(send_request);

					memcpy(requestBuffer, &send_request, nSendMessageSize);
					memcpy(requestBuffer + nSendMessageSize, messageBuffer, sizeof(messageBuffer));

					cout << "[SEND | VS_PROGRAM] Seq: " << nSequenceNumber << " Len: " << nSize
					     << " Message: " << programRequest.id() << " - " << programRequest.name() << endl;
					// cout << "--->" << initRequest.DebugString() << endl;
					stream->send(requestBuffer, sizeof(requestBuffer));
				}
				delete (stream);
			}
		}
		/*if (cvui::button(frame, 300, 110, "Get VS System Status"))
		{
		    stream = connector->connect(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);
		    if (stream)
		    {
		        SystemStatusRequest systemStatusRequest;
		        systemStatusRequest.set_process(VS_SENSOR);
		        systemStatusRequest.set_system_id(1);

		        size_t nSize = systemStatusRequest.ByteSize();
		        systemStatusRequest.SerializeToArray(&messageBuffer[0], nSize);

		        send_request.nSequneceNumber = ++nSequenceNumber;
		        send_request.nLength = nSize;
		        send_request.nCommandIdentifier = VS_SYSTEM_STATUS;

		        int nMessageSize = sizeof(send_request);

		        memcpy(requestBuffer, &send_request, nMessageSize);
		        memcpy(requestBuffer + nMessageSize, messageBuffer, sizeof(messageBuffer));

		        cout << "[SEND | VS_SYSTEM_STATUS] Seq: " << nSequenceNumber << " Len: " << nSize << " Message: " <<
		systemStatusRequest.process() << endl;
		        //cout << "--->" << initRequest.DebugString() << endl;
		        stream->send(requestBuffer, sizeof(requestBuffer));

		        int nSocketSize = 0;
		        int nBytesToRead = sizeof(send_response);

		        // zero buffer
		        memset(responseBuffer, 0x00, sizeof(responseBuffer));
		        //read header
		        while (nBytesToRead > 0)
		        {
		            nSocketSize = stream->receive(responseBuffer, nBytesToRead, true);
		            nBytesToRead -= nSocketSize;
		        }

		        cout << "[RECEIVE] Headersize: " << nSocketSize << endl;

		        memcpy(&send_response, &responseBuffer, sizeof(send_response));
		        if (send_response.nStartIdentifier != VS_MESSAGE_STARTIDENTIFIER)
		        {
		            cout << "[RECEIVE] Unknown StartIdentifier " << (int)send_response.nStartIdentifier << endl;
		            delete stream;
		            exit(-1);
		        }
		        int nSequenceNumber = send_response.nSequneceNumber;
		        int nMessageLength = send_response.nLength;
		        int nCommandIdentifier = send_response.nCommandIdentifier;
		        int nStatus = send_response.nStatus;
		        cout << "[RECEIVE] StartIdentifier " << send_response.nStartIdentifier << " Seq: " <<
		send_response.nSequneceNumber << " Len: " << send_response.nLength << endl;

		        // read body message with length from header
		        while (nMessageLength > 0)
		        {
		            nSocketSize = stream->receive(messageBuffer, nMessageLength, true);
		            nMessageLength -= nSocketSize;
		        }

		        if (send_response.nStatus != ACK)
		        {
		            checkStatus(send_response.nStatus, messageBuffer);
		        }
		        else
		        {
		            cout << "[RECEIVE] VS_SYSTEM_STATUS" << endl;
		            SystemStatusResponse systemStatusResponse;
		            // Read message
		            systemStatusResponse.ParseFromString(messageBuffer);

		            cout << "State: " << systemStatusResponse.state() << endl;

		            if (systemStatusResponse.error_message_size() <= 0)
		                cout << "No, errors in system" << endl;

		            for (int err = 0; err < systemStatusResponse.error_message_size(); err++)
		            {
		                ErrorMessages errorMessage = systemStatusResponse.error_message(err);

		                cout << "Error: " << errorMessage.id() << endl;
		                cout << "\t  " << errorMessage.the_message() << endl;
		            }
		        }
		    }
		    delete (stream);
		}*/

		if (cvui::button(frame, 580, 110, "Cam. Livebild"))
		{
			stream = connector->connect(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);
			if (stream)
			{
				CameraLiveImageRequest cameraLiveImageRequest;
				cameraLiveImageRequest.set_camera_id(1);
				cameraLiveImageRequest.set_live_image(bCameraLiveImage);

				size_t nSize = cameraLiveImageRequest.ByteSize();
				cameraLiveImageRequest.SerializeToArray(&messageBuffer[0], nSize);

				send_request.nSequneceNumber = ++nSequenceNumber;
				send_request.nLength = nSize;
				send_request.nCommandIdentifier = VS_CAMERA_LIVE_IMAGE;

				int nSendMessageSize = sizeof(send_request);

				memcpy(requestBuffer, &send_request, nSendMessageSize);
				memcpy(requestBuffer + nSendMessageSize, messageBuffer, sizeof(messageBuffer));

				stream->send(requestBuffer, sizeof(requestBuffer));
				cout << "[SEND | VS_CAMERA_LIVE_IMAGE] Seq: " << nSequenceNumber << " set live image "
				     << bCameraLiveImage << endl;

				if (bCameraLiveImage)
				{
					bCameraLiveImage = false;
				}
				else
				{
					bCameraLiveImage = true;
				}
				delete (stream);
			}
		}

		if (cvui::button(frame, 480, 110, "VS Livebild"))
		{
			stream = connector->connect(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);
			if (stream)
			{
				VisionSystemLiveImageRequest vsLiveImageRequest;
				vsLiveImageRequest.set_vision_system_id(1);
				vsLiveImageRequest.set_live_image(bVSLiveImage);

				size_t nSize = vsLiveImageRequest.ByteSize();
				vsLiveImageRequest.SerializeToArray(&messageBuffer[0], nSize);

				send_request.nSequneceNumber = ++nSequenceNumber;
				send_request.nLength = nSize;
				send_request.nCommandIdentifier = VS_VISION_SYSTEM_LIVE_IMAGE;

				int nSendMessageSize = sizeof(send_request);

				memcpy(requestBuffer, &send_request, nSendMessageSize);
				memcpy(requestBuffer + nSendMessageSize, messageBuffer, sizeof(messageBuffer));

				stream->send(requestBuffer, sizeof(requestBuffer));
				cout << "[SEND | VS_VISION_SYSTEM_LIVE_IMAGE] Seq: " << nSequenceNumber << " set live image "
				     << bVSLiveImage << endl;

				if (bVSLiveImage)
				{
					bVSLiveImage = false;
				}
				else
				{
					bVSLiveImage = true;
				}
				delete (stream);
			}
		}

		cvui::counter(frame, 630, 150, &nAppData);

		if (cvui::button(frame, 300, 150, "App Data (get)"))
		{
			stream = connector->connect(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);
			if (stream)
			{
				AppData appData;
				appData.set_app_id(nAppData);
				appData.set_vision_system_id(1);

				size_t nSize = appData.ByteSize();
				appData.SerializeToArray(&messageBuffer[0], nSize);

				send_request.nSequneceNumber = ++nSequenceNumber;
				send_request.nLength = nSize;
				send_request.nCommandIdentifier = VS_APP_DATA_GET;

				int nMessageSize = sizeof(send_request);

				memcpy(requestBuffer, &send_request, nMessageSize);
				memcpy(requestBuffer + nMessageSize, messageBuffer, sizeof(messageBuffer));

				cout << "[SEND | VS_APP_DATA_GET] Seq: " << nSequenceNumber << " Len: " << nSize
				     << " Message: " << appData.app_id() << endl;
				stream->send(requestBuffer, sizeof(requestBuffer));

				int nSocketSize = 0;
				int nBytesToRead = sizeof(send_response);

				// zero buffer
				memset(responseBuffer, 0x00, sizeof(responseBuffer));
				// read header
				while (nBytesToRead > 0)
				{
					nSocketSize = stream->receive(responseBuffer, nBytesToRead, true);
					nBytesToRead -= nSocketSize;
				}

				cout << "[RECEIVE] Headersize: " << nSocketSize << endl;

				memcpy(&send_response, &responseBuffer, sizeof(send_response));
				if (send_response.nStartIdentifier != VS_MESSAGE_STARTIDENTIFIER)
				{
					cout << "[RECEIVE] Unknown StartIdentifier " << (int)send_response.nStartIdentifier << endl;
					delete stream;
					exit(-1);
				}
				int nSequenceNumber = send_response.nSequneceNumber;
				int nMessageLength = send_response.nLength;
				int nCommandIdentifier = send_response.nCommandIdentifier;
				int nStatus = send_response.nStatus;
				cout << "[RECEIVE] StartIdentifier " << send_response.nStartIdentifier
				     << " Seq: " << send_response.nSequneceNumber << " Len: " << send_response.nLength << endl;

				// read body message with length from header
				while (nMessageLength > 0)
				{
					nSocketSize = stream->receive(messageBuffer, nMessageLength, true);
					nMessageLength -= nSocketSize;
				}

				if (send_response.nStatus != ACK)
				{
					checkStatus(send_response.nStatus, messageBuffer);
				}
				else
				{
					cout << "[RECEIVE] VS_APP_DATA_GET" << endl;
					AppData appData;
					// Read message
					appData.ParseFromString(messageBuffer);
					cout << appData.DebugString().c_str() << "-----------------------" << endl;
				}
			}
			delete (stream);
		}
		if (cvui::button(frame, 440, 150, "App Data (set)"))
		{
			stream = connector->connect(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);
			if (stream)
			{
				AppData appData;
				appData.set_app_id(nAppData);
				appData.set_vision_system_id(1);

				AppParameterInt* appParameterInt = appData.add_app_parameter_int();
				appParameterInt->set_id(2);
				appParameterInt->set_value(6666);

				AppParameterBool* appParameterBool = appData.add_app_parameter_bool();
				appParameterBool->set_id(0);
				appParameterBool->set_value(true);

				AppParameterBool* appParameterBool2 = appData.add_app_parameter_bool();
				appParameterBool2->set_id(1);
				appParameterBool2->set_value(false);

				AppParameterString* appParameterString = appData.add_app_parameter_string();
				appParameterString->set_id(5);
				appParameterString->set_value("Test der parameter");

				AppParameterFloat* appParameterFloat = appData.add_app_parameter_float();
				appParameterFloat->set_id(4);
				appParameterFloat->set_value(0.6666);

				size_t nSize = appData.ByteSize();
				appData.SerializeToArray(&messageBuffer[0], nSize);

				cout << appData.DebugString().c_str() << "SET:-----------------------" << endl;

				send_request.nSequneceNumber = ++nSequenceNumber;
				send_request.nLength = nSize;
				send_request.nCommandIdentifier = VS_APP_DATA_SET;

				int nMessageSize = sizeof(send_request);

				memcpy(requestBuffer, &send_request, nMessageSize);
				memcpy(requestBuffer + nMessageSize, messageBuffer, sizeof(messageBuffer));

				cout << "[SEND | VS_APP_DATA_SET] Seq: " << nSequenceNumber << " Len: " << nSize
				     << " Message: " << appData.app_id() << endl;
				stream->send(requestBuffer, sizeof(requestBuffer));
			}
			delete (stream);
		}

		cvui::counter(frame, 630, 190, &nCalibration);

		if (cvui::button(frame, 300, 190, "Calibration start"))
		{
			stream = connector->connect(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);
			if (stream)
			{
				CalibrationStartRequest calibrationRequest;
				calibrationRequest.set_vision_system_id(nCalibration);

				size_t nSize = calibrationRequest.ByteSize();
				calibrationRequest.SerializeToArray(&messageBuffer[0], nSize);

				send_request.nSequneceNumber = ++nSequenceNumber;
				send_request.nLength = nSize;
				send_request.nCommandIdentifier = VS_CALIBRATION_START;

				int nMessageSize = sizeof(send_request);

				memcpy(requestBuffer, &send_request, nMessageSize);
				memcpy(requestBuffer + nMessageSize, messageBuffer, sizeof(messageBuffer));

				cout << "[SEND | VS_CALIBRATION_START] Seq: " << nSequenceNumber << " Len: " << nSize
				     << " Message: " << calibrationRequest.vision_system_id() << endl;

				stream->send(requestBuffer, sizeof(requestBuffer));
			}
			delete (stream);
		}

		if (cvui::button(frame, 440, 190, "Calibration end"))
		{
			stream = connector->connect(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);
			if (stream)
			{
				CalibrationStartRequest calibrationRequest;
				calibrationRequest.set_vision_system_id(nCalibration);

				size_t nSize = calibrationRequest.ByteSize();
				calibrationRequest.SerializeToArray(&messageBuffer[0], nSize);

				send_request.nSequneceNumber = ++nSequenceNumber;
				send_request.nLength = nSize;
				send_request.nCommandIdentifier = VS_CALIBRATION_END;

				int nMessageSize = sizeof(send_request);

				memcpy(requestBuffer, &send_request, nMessageSize);
				memcpy(requestBuffer + nMessageSize, messageBuffer, sizeof(messageBuffer));

				cout << "[SEND | VS_CALIBRATION_END] Seq: " << nSequenceNumber << " Len: " << nSize
				     << " Message: " << calibrationRequest.vision_system_id() << endl;

				stream->send(requestBuffer, sizeof(requestBuffer));

				int nSocketSize = 0;
				int nBytesToRead = sizeof(send_response);

				// zero buffer
				memset(responseBuffer, 0x00, sizeof(responseBuffer));
				// read header
				while (nBytesToRead > 0)
				{
					nSocketSize = stream->receive(responseBuffer, nBytesToRead, true);
					nBytesToRead -= nSocketSize;
				}

				cout << "[RECEIVE] Headersize: " << nSocketSize << endl;

				memcpy(&send_response, &responseBuffer, sizeof(send_response));
				if (send_response.nStartIdentifier != VS_MESSAGE_STARTIDENTIFIER)
				{
					cout << "[RECEIVE] Unknown StartIdentifier " << (int)send_response.nStartIdentifier << endl;
					delete stream;
					exit(-1);
				}
				int nSequenceNumber = send_response.nSequneceNumber;
				int nMessageLength = send_response.nLength;
				int nCommandIdentifier = send_response.nCommandIdentifier;
				int nStatus = send_response.nStatus;
				cout << "[RECEIVE] StartIdentifier " << send_response.nStartIdentifier
				     << " Seq: " << send_response.nSequneceNumber << " Len: " << send_response.nLength << endl;

				// read body message with length from header
				while (nMessageLength > 0)
				{
					nSocketSize = stream->receive(messageBuffer, nMessageLength, true);
					nMessageLength -= nSocketSize;
				}

				if (send_response.nStatus != ACK)
				{
					checkStatus(send_response.nStatus, messageBuffer);
				}
				else
				{
					cout << "[RECEIVE] VS_CALIBRATION_END" << endl;
					CalibrationEndResponse calibrationEndResponse;
					// Read message
					calibrationEndResponse.ParseFromString(messageBuffer);
					cout << calibrationEndResponse.DebugString().c_str() << "-----------------------" << endl;
				}
			}
			delete (stream);
		}

		if (cvui::button(frame, 570, 190, "F"))
		{
			stream = connector->connect(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);
			if (stream)
			{
				CalibrationFrameRequest calibrationRequestFrame;
				calibrationRequestFrame.set_vision_system_id(nCalibration);

				size_t nSize = calibrationRequestFrame.ByteSize();
				calibrationRequestFrame.SerializeToArray(&messageBuffer[0], nSize);

				send_request.nSequneceNumber = ++nSequenceNumber;
				send_request.nLength = nSize;
				send_request.nCommandIdentifier = VS_CALIBRATION_FRAME;

				int nMessageSize = sizeof(send_request);

				memcpy(requestBuffer, &send_request, nMessageSize);
				memcpy(requestBuffer + nMessageSize, messageBuffer, sizeof(messageBuffer));

				cout << "[SEND | VS_CALIBRATION_FRAME] Seq: " << nSequenceNumber << " Len: " << nSize
				     << " Message: " << calibrationRequestFrame.vision_system_id() << endl;

				stream->send(requestBuffer, sizeof(requestBuffer));
			}
			delete (stream);
		}

		cvui::counter(frame, 630, 230, &nMarker);

		if (cvui::button(frame, 300, 230, "Marker start"))
		{
			stream = connector->connect(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);
			if (stream)
			{
				MarkerPosition markerPosition;
				markerPosition.set_vision_system_id(nMarker);

				size_t nSize = markerPosition.ByteSize();
				markerPosition.SerializeToArray(&messageBuffer[0], nSize);

				send_request.nSequneceNumber = ++nSequenceNumber;
				send_request.nLength = nSize;
				send_request.nCommandIdentifier = VS_MARKER_POSITION_START;

				int nMessageSize = sizeof(send_request);

				memcpy(requestBuffer, &send_request, nMessageSize);
				memcpy(requestBuffer + nMessageSize, messageBuffer, sizeof(messageBuffer));

				cout << "[SEND | VS_MARKER_START] Seq: " << nSequenceNumber << " Len: " << nSize
				     << " Message: " << markerPosition.vision_system_id() << endl;

				stream->send(requestBuffer, sizeof(requestBuffer));
			}
			delete (stream);
		}

		if (cvui::button(frame, 440, 230, "Marker end"))
		{
			stream = connector->connect(VS_C_DEFAULT_IP, VS_C_SERVER_PORT);
			if (stream)
			{
				MarkerPosition markerPosition;
				markerPosition.set_vision_system_id(nMarker);

				size_t nSize = markerPosition.ByteSize();
				markerPosition.SerializeToArray(&messageBuffer[0], nSize);

				send_request.nSequneceNumber = ++nSequenceNumber;
				send_request.nLength = nSize;
				send_request.nCommandIdentifier = VS_MARKER_POSITION_END;

				int nMessageSize = sizeof(send_request);

				memcpy(requestBuffer, &send_request, nMessageSize);
				memcpy(requestBuffer + nMessageSize, messageBuffer, sizeof(messageBuffer));

				cout << "[SEND | VS_MARKER_END] Seq: " << nSequenceNumber << " Len: " << nSize
				     << " Message: " << markerPosition.vision_system_id() << endl;

				stream->send(requestBuffer, sizeof(requestBuffer));
			}
			delete (stream);
		}

		cvui::update();

		// Show everything on the screen
		cv::imshow(argv[0], frame);

		// Check if ESC key was pressed
		if (cv::waitKey(20) == 27)
		{
			cv::destroyAllWindows();

			exit(0);
		}
	}  // while(1)
	cv::destroyAllWindows();

	exit(0);
}
