/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "udpReceiver.h"

#include <unistd.h>

#include <iostream>

#include "dbFacade/dbFacade.h"

UdpReceiver::UdpReceiver(const std::string& listeningIpAdress, const size_t listeningUdpPort)
    : m_listeningIpAdress(listeningIpAdress), m_listeningUdpPort(listeningUdpPort)
{
	m_socket.open(boost::asio::ip::udp::v4());

	std::shared_ptr<db::DbFacade> m_dbFacade = std::make_shared<db::DbFacade>(
	    "../../config/visionSystemConfig.json");  // TODO(aschaefer): use varibale with path

	bool connectedToTC3 = false;
	while (!connectedToTC3)  // maybe tc3 is booting or something else
	{
		if (m_dbFacade->getShutDownProcesses())
		{
			break;
		}

		try
		{
			m_kLogger->debug("bind to {} / {}", m_listeningIpAdress, m_listeningUdpPort);
			m_socket.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(m_listeningIpAdress),
			                                             m_listeningUdpPort));
			connectedToTC3 = true;
		}
		catch (const std::exception& e)
		{
			m_kLogger->error(e.what());
			m_kLogger->error("{} / {}", m_listeningIpAdress, m_listeningUdpPort);
			m_kLogger->error("repeat trying to bind to visionSystem IP/Port for tc3 communication ...");
			sleep(1);
		}
	}
}

void UdpReceiver::stopIOService() { m_ioService.stop(); }

void UdpReceiver::parseTc3ToVsDatagram(boost::array<unsigned char, kTc3ToVsBufferSize> buffer)
{
	std::memcpy(&m_recObj.sHeader.sVersion.uMainVersion, &buffer[0], 4);
	std::memcpy(&m_recObj.sHeader.sVersion.uMinorVersion, &buffer[4], 4);
	std::memcpy(&m_recObj.sHeader.sVersion.uRevision, &buffer[8], 4);
	std::memcpy(&m_recObj.sHeader.uWatchdogCounter, &buffer[12], 4);
	std::memcpy(&m_recObj.sCommand.eActualCommandVSComm, &buffer[16], 4);
	std::memcpy(&m_recObj.sCameraTrigger.uImageNumber, &buffer[20], 8);
	std::memcpy(&m_recObj.sCameraTrigger.uActValRotEncoderConveyor, &buffer[28], 4);
	std::memcpy(&m_recObj.sCameraTrigger.uYear, &buffer[32], 4);
	std::memcpy(&m_recObj.sCameraTrigger.uMonth, &buffer[36], 4);
	std::memcpy(&m_recObj.sCameraTrigger.uDay, &buffer[40], 4);
	std::memcpy(&m_recObj.sCameraTrigger.uHour, &buffer[44], 4);
	std::memcpy(&m_recObj.sCameraTrigger.uMinute, &buffer[48], 4);
	std::memcpy(&m_recObj.sCameraTrigger.uSecond, &buffer[52], 4);
	std::memcpy(&m_recObj.sCameraTrigger.uMilliSecond, &buffer[56], 4);
}

void UdpReceiver::handleReceive(const boost::system::error_code& error, size_t bytes_transferred)
{
	if (error)
	{
		m_kLogger->error("Receive failed: {}", error.message());
		return;
	}

	m_bytesTransferred = bytes_transferred;
	parseTc3ToVsDatagram(m_recvBuffer);
	m_bDataReceived = true;
	wait();  // wait for next message
}

void UdpReceiver::wait()
{
	m_socket.async_receive_from(boost::asio::buffer(m_recvBuffer),
	                            m_remoteEndpoint,
	                            boost::bind(&UdpReceiver::handleReceive,
	                                        this,
	                                        boost::asio::placeholders::error,
	                                        boost::asio::placeholders::bytes_transferred));
}

void UdpReceiver::receive()
{
	wait();  // waiting  asynchronously for incoming message then start callback
	m_ioService.run();
}
