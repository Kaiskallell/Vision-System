#include "connectionHandler.h"

#include <fstream>

#include "Nack.pb.h"
#include "projectPaths.h"
#include "vs_c_format.h"
#include "vs_c_init.h"
#include "vs_c_liveImage.h"
#include "vs_c_marker.h"
#include "vs_c_productionState.h"
#include "vs_c_status.h"
#include "vs_c_tcpAcceptor.h"
#include "vs_c_thread.h"
#include "vs_c_versionCheckVsPs.h"

using namespace VMS::VisionSystem;  // protofiles

GetStatus getStatus;

void ConnectionHandler::vsToVmsSend(TCPStream* stream,
                                    const std::string& sMessageBuffer,
                                    const eCommandIdentifier eCommandID,
                                    const int nSequenceNo,
                                    const int nSize,
                                    const eResponseStatus status)
{
	// class declaration send
	vs_message_header_response responseHeader;

	// Set Header
	responseHeader.nCommandIdentifier = eCommandID;
	responseHeader.nSequneceNumber = nSequenceNo;
	responseHeader.nLength = nSize;
	responseHeader.nStatus = status;  // ACK or NACK

	int responseHeaderSize = sizeof(vs_message_header_response);
	char sSendBuffer[responseHeaderSize + sMessageBuffer.size()];
	memcpy(sSendBuffer, &responseHeader, responseHeaderSize);  // writing header of msg
	memcpy(sSendBuffer + responseHeaderSize,
	       sMessageBuffer.data(),
	       sMessageBuffer.size());  // writing body of msg (protobuf)

	stream->send(sSendBuffer, sizeof(sSendBuffer));
}

// send NACK message to ST
void ConnectionHandler::vsSendNack(TCPStream* stream,
                                   eCommandIdentifier eComandID,
                                   int nSequenceNo,
                                   eResponseNACK_Errors eError)
{
	std::string msgBody = "";
	ResponseNack responseNack;
	responseNack.set_error_code((uint32_t)eError);
	size_t nSize = responseNack.ByteSize();
	responseNack.SerializeToString(&msgBody);
	vsToVmsSend(stream, msgBody, eComandID, nSequenceNo, responseNack.ByteSize(), NACK);
}

ConnectionHandler::ConnectionHandler(vsQueue<WorkItem*>& queue) : m_queue(queue) {

}

void* ConnectionHandler::run()
{
	// send receive buffer
	char receiveBuffer[VS_MESSAGE_BUFFER_SIZE];
	char sendBuffer[VS_MESSAGE_BUFFER_SIZE];
	// body area
	char messageBuffer[VS_MESSAGE_BUFFER_SIZE];
	char sendMessageBuffer[VS_MESSAGE_BUFFER_SIZE];

	// class declaration send/receive
	vs_message_header_request send_request_header;

	int nReceivedMessageLength = 0;
	int32_t nSequenceNumber = 0;
	string result;

	int nCommandIdentifier;

	memset(receiveBuffer, 0xFF, sizeof(receiveBuffer));
	// Remove 1 item at a time and process it.
	// Blocks if no items are available to process
	for (int i = 0;; i++)
	{
		WorkItem* item = m_queue.remove();  // returns first element in list
		TCPStream* stream = item->getStream();

		// clear buffer
		memset(messageBuffer, 0x00, sizeof(messageBuffer));
		memset(sendBuffer, 0x00, sizeof(sendBuffer));
		memset(receiveBuffer, 0x00, sizeof(receiveBuffer));
		memset(sendMessageBuffer, 0x00, sizeof(sendMessageBuffer));

		result.clear();

		int nReceivedSocketSize = 0;
		int nBytesToRead = sizeof(send_request_header);

		// read header
		while (nBytesToRead > 0)
		{
			nReceivedSocketSize = stream->receive(receiveBuffer, nBytesToRead, true);
			nBytesToRead -= nReceivedSocketSize;
		}
		// copy buffer in send_request_header
		memcpy(&send_request_header, &receiveBuffer, sizeof(send_request_header));

		int nStartIdentifier = (int)send_request_header.nStartIdentifier;

		if (nStartIdentifier == VS_MESSAGE_STARTIDENTIFIER)
		{
			nSequenceNumber = send_request_header.nSequneceNumber;
			nReceivedMessageLength = send_request_header.nLength;
			nCommandIdentifier = send_request_header.nCommandIdentifier;
		}
		else
		{
			m_kLogger->warn("[RECEIVE] Unknown StartIdentifier {}", nStartIdentifier);
			break;
		}

		// read body message with length from header
		while (nReceivedMessageLength > 0)
		{
			nReceivedSocketSize = stream->receive(messageBuffer, nReceivedMessageLength, true);
			nReceivedMessageLength -= nReceivedSocketSize;
		}
		string sMessageBuffer(messageBuffer, send_request_header.nLength);

		switch (nCommandIdentifier)
		{
			default: {
				m_kLogger->error("[RECEIVE] Unknown CommandIdentifier {}", send_request_header.nCommandIdentifier);
				vsSendNack(stream, VS_INIT_REQUEST, nSequenceNumber, PB_ERR_UNKNOWN_COMMAND_ID);
				break;
			}
			case VS_INIT_REQUEST: {
				m_kLogger->info("[RECEIVE] VS_INIT_REQUEST");
				constexpr int kRobotTypeTog519 = 0;
				Init init(kRobotTypeTog519);

				int nReturn = init.processInit(sMessageBuffer, result);
				if (nReturn == init.InitParsing)
				{
					vsSendNack(stream, VS_INIT_REQUEST, nSequenceNumber, PB_ERR_PARSING);
				}
				else if (nReturn == init.InitVersion)
				{
					vsSendNack(stream, VS_INIT_REQUEST, nSequenceNumber, VS_ERR_WRONG_VERSION);
				}
				else if (nReturn == init.InitRobotTypeWrong)
				{
					vsSendNack(stream, VS_INIT_REQUEST, nSequenceNumber, VS_ERR_WRONG_ROBOT_TYPE);
				}
				else if (nReturn == init.InitOk)
				{
					vsToVmsSend(stream, result, VS_INIT_REQUEST, nSequenceNumber, init.getMessageSize(), ACK);
				}
				else
				{
					vsSendNack(stream, VS_INIT_REQUEST, nSequenceNumber, PB_ERR_PARSING);
				}

				break;
			}

			case VS_VERSION_CHECK: {
				m_kLogger->info("[RECEIVE] VS_VERSION_CHECK");
				VersionCheckVsPs versionCheckVsPs;

				int nReturn = versionCheckVsPs.processVersionCheckVsPs(sMessageBuffer, result);
				if (nReturn == versionCheckVsPs.VersionCheckVsPsParsing)
				{
					m_kLogger->error("[SEND] NACK VS_VERSION_CHECK: VsPsParsing");
					vsToVmsSend(
						stream, result, VS_VERSION_CHECK, nSequenceNumber, versionCheckVsPs.getMessageSize(), NACK);
				}
				else if (nReturn == versionCheckVsPs.VersionCheckVsPsVersion)
				{
					m_kLogger->error("[SEND] NACK VS_VERSION_CHECK: PsVersion");
					vsToVmsSend(
						stream, result, VS_VERSION_CHECK, nSequenceNumber, versionCheckVsPs.getMessageSize(), NACK);
				}
				else if (nReturn == versionCheckVsPs.VersionCheckVsPsOk)
				{
					m_kLogger->info("[SEND] VS_VERSION_CHECK");
					vsToVmsSend(
						stream, result, VS_VERSION_CHECK, nSequenceNumber, versionCheckVsPs.getMessageSize(), ACK);
				}
				else
				{
					m_kLogger->error("[SEND] NACK VS_VERSION_CHECK");
					vsToVmsSend(
						stream, result, VS_VERSION_CHECK, nSequenceNumber, versionCheckVsPs.getMessageSize(), NACK);
				}
				break;
			}
			case VS_GET_STATUS: {
				int nReturn = getStatus.processGetStatus(sMessageBuffer, result);
				if (nReturn == PB_ERR_PARSING)
				{
					vsSendNack(stream, VS_GET_STATUS, nSequenceNumber, PB_ERR_PARSING);
				}
				else
				{
					vsToVmsSend(stream, result, VS_GET_STATUS, nSequenceNumber, getStatus.getMessageSize(), ACK);
				}
				break;
			}
			case VS_START_PRODUCTION: {
				m_kLogger->info("[RECEIVE] VS_START_PRODUCTION");
				ProductionState startProduction;
				int nReturn = startProduction.processStartProduction(sMessageBuffer, result);
				if (nReturn == startProduction.StartProductionParsing)
				{
					vsSendNack(stream, VS_START_PRODUCTION, nSequenceNumber, PB_ERR_PARSING);
				}
				else if (nReturn == startProduction.StartProductionWrongID)
				{
					vsSendNack(stream, VS_START_PRODUCTION, nSequenceNumber, VS_ERR_UNKNOWN_APP_ID);
				}
				else if (nReturn == startProduction.StartProductionIDNotInDB)
				{
					vsSendNack(stream, VS_START_PRODUCTION, nSequenceNumber, VS_ERR_UNKNOWN_DATABASE_ENTRY);
				}
				else if (nReturn == startProduction.StartProductionOk)
				{
					vsToVmsSend(
						stream, result, VS_START_PRODUCTION, nSequenceNumber, startProduction.getMessageSize(), ACK);
				}
				break;
			}
			case VS_FORMAT: {
				m_kLogger->info("[RECEIVE] VS_FORMAT");
				Format format;
				int nReturn = format.processFormat(sMessageBuffer, result);
				if (nReturn == format.FormatParsing)
				{
					vsSendNack(stream, VS_FORMAT, nSequenceNumber, PB_ERR_PARSING);
				}
				else if ((nReturn == format.FormatFileNotFound) || (nReturn == format.FormatDirNotFound)
						|| (nReturn == format.FormatUnknown))
				{
					std::string msgBody = "";
					int size = 0;
					vsToVmsSend(stream, msgBody, VS_FORMAT, nSequenceNumber, size, NACK);
				}
				else if (nReturn == format.FormatOk)
				{
					m_kLogger->debug("FormatOk vs_sendAck: {}/{}", nSequenceNumber, format.getMessageSize());
					vsToVmsSend(stream, result, VS_FORMAT, nSequenceNumber, format.getMessageSize(), ACK);
				}
				break;
			}
			case VS_FORMAT_AVAILABLE: {
				m_kLogger->info("[RECEIVE] VS_FORMAT_AVAILABLE");
				Format format;
				if (!format.processFormatAvailable(sMessageBuffer, result))
				{
					vsSendNack(stream, VS_FORMAT_AVAILABLE, nSequenceNumber, VS_ERR_FORMAT_FILE_NOT_FOUND);
				}
				else
				{
					vsToVmsSend(stream, result, VS_FORMAT_AVAILABLE, nSequenceNumber, format.getMessageSize(), ACK);
				}
				break;
			}
			case VS_FORMAT_LOADED: {
				m_kLogger->info("[RECEIVE] VS_FORMAT_LOADED");
				Format format;
				if (!format.processFormatLoaded(sMessageBuffer, result))
				{
					vsSendNack(stream, VS_FORMAT_LOADED, nSequenceNumber, VS_ERR_UNKNOWN_DATABASE_ENTRY);
				}
				else
				{
					vsToVmsSend(stream, result, VS_FORMAT_LOADED, nSequenceNumber, format.getMessageSize(), ACK);
				}

				break;
			}
			case VS_MARKER_POSITION_GET: {
				m_kLogger->info("[RECEIVE] VS_MARKER_POSITION_GET");
				MarkerClass markerGet;
				if (!markerGet.processMarkerGet(sMessageBuffer, result))
				{
					vsSendNack(stream, VS_MARKER_POSITION_GET, nSequenceNumber, PB_ERR_PARSING);
				}
				else
				{
					vsToVmsSend(
						stream, result, VS_MARKER_POSITION_GET, nSequenceNumber, markerGet.getMessageSize(), ACK);
				}

				break;
			}
			case VS_MARKER_POSITION_SET: {
				m_kLogger->info("[RECEIVE] VS_MARKER_POSITION_SET");
				MarkerClass markerSet;
				if (!markerSet.processMarkerSet(sMessageBuffer, result))
				{
					vsSendNack(stream, VS_MARKER_POSITION_SET, nSequenceNumber, PB_ERR_PARSING);
				}
				break;
			}
			case VS_MARKER_POSITION_START: {
				m_kLogger->info("[RECEIVE] VS_MARKER_POSITION_START");
				MarkerClass markerStart;
				if (!markerStart.processMarkerStart(sMessageBuffer, result))
				{
					vsSendNack(stream, VS_MARKER_POSITION_START, nSequenceNumber, PB_ERR_PARSING);
				}
				break;
			}
			case VS_MARKER_POSITION_END: {
				m_kLogger->info("[RECEIVE] VS_MARKER_POSITION_END");
				MarkerClass markerEnd;
				if (!markerEnd.processMarkerEnd(sMessageBuffer, result))
				{
					vsSendNack(stream, VS_MARKER_POSITION_END, nSequenceNumber, PB_ERR_PARSING);
				}

				break;
			}
			case VS_CAMERA_LIVE_IMAGE: {
				m_kLogger->info("[RECEIVE] VS_CAMERA_LIVE_IMAGE");
				LiveImage liveImageCamera;
				if (!liveImageCamera.processLiveImageCamera(sMessageBuffer, result))
				{
					vsSendNack(stream, VS_CAMERA_LIVE_IMAGE, nSequenceNumber, PB_ERR_PARSING);
				}
				break;
			}
			case VS_VISION_SYSTEM_LIVE_IMAGE: {
				m_kLogger->info("[RECEIVE] VS_VISION_SYSTEM_LIVE_IMAGE");
				LiveImage liveImageVisionSystem;
				if (!liveImageVisionSystem.processLiveImageVisionSystem(sMessageBuffer, result))
				{
					vsSendNack(stream, VS_VISION_SYSTEM_LIVE_IMAGE, nSequenceNumber, PB_ERR_PARSING);
				}
				break;
			}
		}  // end switch case
		
		

		delete item;
	}  // for
	// Should never get here
	return NULL;
}
