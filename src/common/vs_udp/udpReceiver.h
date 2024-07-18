/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef UDPRECEIVER_H
#define UDPRECEIVER_H

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include "logging.h"
#include "udpCommStructs2VS.h"

class UdpReceiver
{
  private:
	boost::asio::io_service m_ioService;
	boost::asio::ip::udp::socket m_socket{m_ioService};

	boost::array<unsigned char, kTc3ToVsBufferSize> m_recvBuffer;
	boost::asio::ip::udp::endpoint m_remoteEndpoint;

	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("UdpReceiver");

  public:
	std::string m_listeningIpAdress = "";
	unsigned int m_listeningUdpPort = 0;

	size_t m_bytesTransferred = 0;
	Tc3ToVsDatagram m_recObj;
	std::atomic<bool> m_bDataReceived = false;

	explicit UdpReceiver(const std::string& listeningIpAdress, const size_t listeningUdpPort);
	~UdpReceiver() = default;

	///
	/// \brief stopIOService stops the boost::asio::io_service.
	/// This needs to be stoped in order to join the receiverthread
	///
	void stopIOService();

	///
	/// \brief parseTc3ToVsDatagram Converts boost::array which is comming from TwinCat3.
	/// \param buffer
	///
	void parseTc3ToVsDatagram(boost::array<unsigned char, kTc3ToVsBufferSize> buffer);

	///
	/// \brief handleReceive It parses the incomming std::array. Then it checks if a
	/// trigger as emmited by TwinCat3 (@var uImageAcqisition). Then the function @ref wait
	/// is called to listen to next message
	/// \param error
	/// \param bytes_transferred
	///
	void handleReceive(const boost::system::error_code& error, size_t bytes_transferred);

	///
	/// \brief wait If a message arrives the function @ref handleReceive is called
	///
	void wait();

	///
	/// \brief receive This function is supposed to be started in a thread
	/// It opens a socket and @ref wait  for a incomming message.
	///
	void receive();
};

#endif  // UDPCLIENT_H
