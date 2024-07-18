/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "tc3Mock.h"

#include <thread>

namespace tc3mock
{
Tc3Mock::Tc3Mock(const std::string& listeningIpAdress,
                 const size_t listeningUdpPort,
                 const std::string& sendingIpAdress,
                 const size_t sendingUdpPort)
    : m_sendingIpAdress(sendingIpAdress), m_sendingUdpPort(sendingUdpPort)
{
	std::cout << "trying to setup a udp receiver for TC3 Mock" << std::endl;
	m_receiver = std::make_shared<Receiver>(listeningIpAdress, listeningUdpPort);

	if (m_receiverThread.joinable())
	{
		m_receiverThread.join();
	}

	m_receiverThread = std::thread([&] { m_receiver->receive(); });

	std::this_thread::sleep_for(std::chrono::microseconds(
	    15000));  // waiting for reciever thread to get set up, before sending data to this port
}

Tc3Mock::~Tc3Mock()
{
	if (m_receiverThread.joinable())
	{
		m_receiver->stopIOService();
		m_receiverThread.join();
	}
}

bool Tc3Mock::receivedData() { return m_receiver->m_bDataReceived; }

VsToTc3Datagram Tc3Mock::getReceiverData() { return m_receiver->getData(); }

boost::array<unsigned char, kTc3ToVsBufferSize> makeTc3ToVsBuffer(const Tc3ToVsDatagram& m_sendObj)
{
	boost::array<unsigned char, kTc3ToVsBufferSize> buffer;
	std::memcpy(&buffer[0], &m_sendObj.sHeader.sVersion.uMainVersion, 4);
	std::memcpy(&buffer[4], &m_sendObj.sHeader.sVersion.uMinorVersion, 4);
	std::memcpy(&buffer[8], &m_sendObj.sHeader.sVersion.uRevision, 4);
	std::memcpy(&buffer[12], &m_sendObj.sHeader.uWatchdogCounter, 4);
	std::memcpy(&buffer[16], &m_sendObj.sCommand.eActualCommandVSComm, 4);
	std::memcpy(&buffer[20], &m_sendObj.sCameraTrigger.uImageNumber, 8);
	std::memcpy(&buffer[28], &m_sendObj.sCameraTrigger.uActValRotEncoderConveyor, 4);
	std::memcpy(&buffer[32], &m_sendObj.sCameraTrigger.uYear, 4);
	std::memcpy(&buffer[36], &m_sendObj.sCameraTrigger.uMonth, 4);
	std::memcpy(&buffer[40], &m_sendObj.sCameraTrigger.uDay, 4);
	std::memcpy(&buffer[44], &m_sendObj.sCameraTrigger.uHour, 4);
	std::memcpy(&buffer[48], &m_sendObj.sCameraTrigger.uMinute, 4);
	std::memcpy(&buffer[52], &m_sendObj.sCameraTrigger.uSecond, 4);
	std::memcpy(&buffer[56], &m_sendObj.sCameraTrigger.uMilliSecond, 4);
	return buffer;
}

void Tc3Mock::send(const Tc3ToVsDatagram& sendObj)
{
	boost::array<unsigned char, kTc3ToVsBufferSize> buffer = makeTc3ToVsBuffer(sendObj);
	m_receiver->m_bDataReceived = false;
	boost::asio::io_service ioService;
	udp::socket socket(ioService);
	udp::endpoint remoteEndpoint = udp::endpoint(address::from_string(m_sendingIpAdress), m_sendingUdpPort);
	socket.open(udp::v4());

	boost::system::error_code err;
	auto sent = socket.send_to(boost::asio::buffer(buffer), remoteEndpoint, 0, err);
	socket.close();
	std::cout << "Sent Payload --- " << sent << "\n";
}
}  // namespace tc3mock
