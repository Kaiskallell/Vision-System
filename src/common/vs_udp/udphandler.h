
/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef UDPHANDLER_H
#define UDPHANDLER_H

#include <iostream>
#include <thread>

#include "dbFacade/dbFacade.h"
#include "logging.h"
#include "opencv2/opencv.hpp"
#include "udpCommStructs2KC.h"
#include "udpReceiver.h"
#include "vs_image.hpp"

class UdpHandler
{
  public:
	UdpHandler(){};
	UdpHandler(const std::string& listeningIpAdress,
	           const size_t listeningUdpPort,
	           std::string& sendingIpAdress,
	           const size_t sendingUdpPort,
	           const int areaNumber,
			   const bool udpEnabled);
	~UdpHandler();
	bool m_configured = false;
	///
	/// \brief sendTo Sends all data which is currently stored in sendingObj
	///
	virtual bool sendTo(eCommandVSComm eCommandVSComm, eProductsSituationVSComm eProductsSituationVSComm);

	virtual bool sendToWithClassId(eCommandVSComm eCommandVSComm,
	                               eProductsSituationVSComm eProductsSituationVSComm,
	                               const uint32_t classId);

	///
	/// \brief waitForMessageUsec Waits for a message from TwinCat or a given @param timeout [usec]
	/// \param timeout
	/// \return Returns true if communication with TwinCat was successful
	///
	bool waitForMessageUsec(unsigned int timeout);

	virtual void checkConnectionToTC3();

	virtual bool askTC3ToMakeTriggerSignalAndGetEncoderVal(uint64_t& imageNumber,
	                                                       uint32_t& rotEncoderVal,
	                                                       TimeStamp& timeStamp,
	                                                       bool& frameUpdated);

	///
	/// \brief m_receiver listens to data comming from TwinCat3
	///
	std::shared_ptr<UdpReceiver> m_receiver;

	///
	/// \brief m_sendRequestDataObj This is the payload for the message which is send to
	/// TwinCat3.
	///
	VsToTc3Datagram m_sendRequestDataObj;
	bool m_udpEnabled = true;


  protected:
	///
	/// \brief startRecieverThread this starts the thread which is listen
	/// to the signals emmited by TWINCAT3
	///
	virtual void startRecieverThread();

	///
	/// \brief parseVsToTc3Datagram converts sendObj to a std::array which is needed
	/// for boost asio and to meet TwinCat3 format standart
	///
	void parseVsToTc3Datagram(VsToTc3Datagram& sendRequestDataObj);

	std::array<unsigned char, 32> m_sendArrayMsg;

	///
	/// \brief m_receiverThread This thread enables the @var _client to listen to
	/// the data which is comming from TwinCat3
	///
	std::thread m_receiverThread;

	std::string m_sendingIpAdress = "";
	unsigned int m_sendingUdpPort = 0;
	bool m_frameNumberOverflow = false;
	int m_areaNumber = 0;
	std::shared_ptr<db::DbFacade> m_dbFacade;
	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("UdpHandler");
};

namespace udpHander
{
std::shared_ptr<UdpHandler> setupUdpHandler(const fs::path& networksConfigPath,
                                            const std::shared_ptr<spdlog::logger> kLogger);
}

#endif  // UDPHANDLER_H
