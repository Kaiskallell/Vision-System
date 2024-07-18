/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef TC3_MOCK_H
#define TC3_MOCK_H

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "udpCommStructs2KC.h"
#include "udpCommStructs2VS.h"

namespace tc3mock
{
using boost::asio::ip::address;
using boost::asio::ip::udp;

class Receiver
{
  public:
	Receiver(const std::string& listeningIpAdress, const size_t listeningUdpPort)
	    : m_listeningIpAdress(listeningIpAdress), m_listeningUdpPort(listeningUdpPort)
	{
		m_socket.open(boost::asio::ip::udp::v4());

		bool connectedToVS = false;
		while (!connectedToVS)  // maybe tc3 is booting or something else
		{
			try
			{
				std::cout << "bind to " << m_listeningIpAdress << " / " << m_listeningUdpPort << std::endl;
				m_socket.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(m_listeningIpAdress),
				                                             m_listeningUdpPort));
				connectedToVS = true;
			}
			catch (const std::exception& e)
			{
				std::cout << e.what() << std::endl;
				std::cout << m_listeningIpAdress << " / " << m_listeningUdpPort << std::endl;
				std::cout << "repeat trying to bind to tc3, maybe it is booting ..." << std::endl;
				sleep(1);
			}
		}
	}

	~Receiver() = default;
	std::atomic<bool> m_bDataReceived = false;

	void receive()
	{
		wait();
		m_ioService.run();
	}

	void stopIOService() { m_ioService.stop(); }
	VsToTc3Datagram getData() { return m_data; };

  private:
	boost::asio::io_service m_ioService;
	udp::socket m_socket{m_ioService};
	boost::array<unsigned char, 32> m_recv_buffer;
	udp::endpoint m_remote_endpoint;
	const std::string m_listeningIpAdress = "";
	const size_t m_listeningUdpPort = 0;
	VsToTc3Datagram m_data;

	void handle_receive(const boost::system::error_code& error, size_t bytes_transferred)
	{
		if (error)
		{
			std::cout << "Receive failed: " << error.message() << "\n";
			return;
		}
		std::memcpy(&m_data.sHeader.sVersion.uMainVersion, &m_recv_buffer[0], 4);
		std::memcpy(&m_data.sHeader.sVersion.uMinorVersion, &m_recv_buffer[4], 4);
		std::memcpy(&m_data.sHeader.sVersion.uRevision, &m_recv_buffer[8], 4);
		std::memcpy(&m_data.sHeader.uWatchdogCounter, &m_recv_buffer[12], 4);
		std::memcpy(&m_data.sHeader.uAreaNumber, &m_recv_buffer[16], 4);
		std::memcpy(&m_data.sCommand.eActualCommandVSComm, &m_recv_buffer[20], 4);
		std::memcpy(&m_data.sProductsSituation.eActualProductsSituationVSComm, &m_recv_buffer[24], 4);
		std::memcpy(&m_data.sProductsSituation.classId, &m_recv_buffer[28], 4);

		m_bDataReceived = true;
		wait();
	}

	void wait()
	{
		m_socket.async_receive_from(boost::asio::buffer(m_recv_buffer),
		                            m_remote_endpoint,
		                            boost::bind(&Receiver::handle_receive,
		                                        this,
		                                        boost::asio::placeholders::error,
		                                        boost::asio::placeholders::bytes_transferred));
	}
};

class Tc3Mock
{
  public:
	Tc3Mock(const std::string& listeningIpAdress,
	        const size_t listeningUdpPort,
	        const std::string& sendingIpAdress,
	        const size_t sendingUdpPort);
	~Tc3Mock();
	void send(const Tc3ToVsDatagram& sendObj);
	bool receivedData();
	VsToTc3Datagram getReceiverData();

  private:
	const std::string m_sendingIpAdress = "";
	const size_t m_sendingUdpPort = 0;

	std::shared_ptr<Receiver> m_receiver;
	std::thread m_receiverThread;
};
}  // namespace tc3mock

#endif