
/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "udphandler.h"

#include "json.hpp"
#include "mockUdpHandler.hpp"
#include "projectPaths.h"

UdpHandler::UdpHandler(const std::string& listeningIpAdress,
                       const size_t listeningUdpPort,
                       std::string& sendingIpAdress,
                       const size_t sendingUdpPort,
                       const int areaNumber,
					   const bool udpEnabled)
{
	m_dbFacade = std::make_shared<db::DbFacade>(
	    "../../config/visionSystemConfig.json");  // TODO(aschaefer): use varibale with path
	m_receiver = std::make_shared<UdpReceiver>(listeningIpAdress, listeningUdpPort);
	m_sendingIpAdress = sendingIpAdress;
	m_sendingUdpPort = sendingUdpPort;
	m_areaNumber = areaNumber;
	m_udpEnabled = udpEnabled;

	this->startRecieverThread();  // construction of receiver thread
}

UdpHandler::~UdpHandler()
{
	// Stop udp thread and close socket
	if (m_receiverThread.joinable())
	{
		m_receiver->stopIOService();
		m_receiverThread.join();
	}
}

void UdpHandler::startRecieverThread()
{
	if (m_receiverThread.joinable())
	{
		m_receiverThread.join();
	}
	m_receiverThread = std::thread([&] { m_receiver->receive(); });
	std::this_thread::sleep_for(std::chrono::microseconds(
	    15000));  // waiting for reciever thread to get set up, before sending data to this port
}

bool UdpHandler::sendTo(eCommandVSComm eCommandVSComm, eProductsSituationVSComm eProductsSituationVSComm)
{
	m_sendRequestDataObj.sHeader.uAreaNumber = m_areaNumber;
	m_sendRequestDataObj.sCommand.eActualCommandVSComm = eCommandVSComm;
	m_sendRequestDataObj.sProductsSituation.eActualProductsSituationVSComm = eProductsSituationVSComm;
	m_sendRequestDataObj.sProductsSituation.classId = 0;

	m_receiver->m_bDataReceived = false;
	m_sendRequestDataObj.sHeader.uWatchdogCounter++;
	// m_sendArrayMsg gets filled
	parseVsToTc3Datagram(m_sendRequestDataObj);

	boost::asio::io_service ioService;
	boost::asio::ip::udp::socket socket(ioService);
	boost::asio::ip::udp::endpoint remoteEndpoint =
	    boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(m_sendingIpAdress), m_sendingUdpPort);
	socket.open(boost::asio::ip::udp::v4());
	boost::system::error_code err;

	auto sent = socket.send_to(boost::asio::buffer(m_sendArrayMsg), remoteEndpoint, 0, err);
	constexpr size_t kTimeoutUsec = 5000000;
	if (waitForMessageUsec(kTimeoutUsec))
	{
		m_sendRequestDataObj.sHeader.uWatchdogCounter = 0;
		return true;  // Time Out for response
	}
	socket.close();

	return false;  // no Time Out for response
}

bool UdpHandler::sendToWithClassId(eCommandVSComm eCommandVSComm,
                                   eProductsSituationVSComm eProductsSituationVSComm,
                                   const uint32_t classId)
{
	m_sendRequestDataObj.sHeader.uAreaNumber = m_areaNumber;
	m_sendRequestDataObj.sCommand.eActualCommandVSComm = eCommandVSComm;
	m_sendRequestDataObj.sProductsSituation.eActualProductsSituationVSComm = eProductsSituationVSComm;
	m_sendRequestDataObj.sProductsSituation.classId = classId;

	m_receiver->m_bDataReceived = false;
	m_sendRequestDataObj.sHeader.uWatchdogCounter++;
	// m_sendArrayMsg gets filled
	parseVsToTc3Datagram(m_sendRequestDataObj);

	boost::asio::io_service ioService;
	boost::asio::ip::udp::socket socket(ioService);
	boost::asio::ip::udp::endpoint remoteEndpoint =
	    boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(m_sendingIpAdress), m_sendingUdpPort);
	socket.open(boost::asio::ip::udp::v4());
	boost::system::error_code err;

	auto sent = socket.send_to(boost::asio::buffer(m_sendArrayMsg), remoteEndpoint, 0, err);
	constexpr size_t kTimeoutUsec = 5000000;
	if (waitForMessageUsec(kTimeoutUsec))
	{
		m_sendRequestDataObj.sHeader.uWatchdogCounter = 0;
		return true;  // Time Out for response
	}
	socket.close();

	return false;  // no time out for response
}

void UdpHandler::checkConnectionToTC3()
{
	if (!m_udpEnabled)
	{
		return;
	}

	m_kLogger->debug("checking connection to TC3");
	// syncronization with twinCat
	while (this->sendTo(eCommandVSComm_SYNC, eProductsSituationVSComm_NOTAVAILABLE))
	{
		if (m_dbFacade->getShutDownProcesses())
		{
			return;
		}
		continue;  // Do nothing until receiving 'false' (no timeout) from TwinCat than go to takeFrame
	}
}

bool UdpHandler::askTC3ToMakeTriggerSignalAndGetEncoderVal(uint64_t& imageNumber,
                                                           uint32_t& rotEncoderVal,
                                                           TimeStamp& timeStamp,
                                                           bool& frameUpdated)
{
	if (!m_udpEnabled)
	{
		frameUpdated = true;
		return true;
	}

	if (this->sendTo(eCommandVSComm_TAKEIMAGE, eProductsSituationVSComm_NOTAVAILABLE))
	{
		checkConnectionToTC3();
		return false;
	}

	// in case of overflow of uint64_t, set it manually to zero
	// otherwise the next if statement would fail
	if (imageNumber == (std::pow(2, 64) - 1))
	{
		m_frameNumberOverflow = true;
		imageNumber = 0;
	}

	// TODO(aschaefer): simplify and invert this condition
	// check if there is a new frame
	if (this->m_receiver->m_recObj.sCameraTrigger.uImageNumber > imageNumber
	    || (m_frameNumberOverflow && this->m_receiver->m_recObj.sCameraTrigger.uImageNumber == 0))
	{
		if (m_frameNumberOverflow)
		{
			m_frameNumberOverflow = false;
		}

		frameUpdated = true;
	}

	// if there is a new frame we take the imageNumber and the encoder value form TwinCat
	imageNumber = this->m_receiver->m_recObj.sCameraTrigger.uImageNumber;
	rotEncoderVal = this->m_receiver->m_recObj.sCameraTrigger.uActValRotEncoderConveyor;
	timeStamp.year = this->m_receiver->m_recObj.sCameraTrigger.uYear;
	timeStamp.day = this->m_receiver->m_recObj.sCameraTrigger.uDay;
	timeStamp.hour = this->m_receiver->m_recObj.sCameraTrigger.uHour;
	timeStamp.minute = this->m_receiver->m_recObj.sCameraTrigger.uMinute;
	timeStamp.second = this->m_receiver->m_recObj.sCameraTrigger.uSecond;
	timeStamp.millisecond = this->m_receiver->m_recObj.sCameraTrigger.uMilliSecond;

	return true;
}

void UdpHandler::parseVsToTc3Datagram(VsToTc3Datagram& sendRequestDataObj)
{
	std::memcpy(&m_sendArrayMsg[0], &sendRequestDataObj.sHeader.sVersion.uMainVersion, 4);
	std::memcpy(&m_sendArrayMsg[4], &sendRequestDataObj.sHeader.sVersion.uMinorVersion, 4);
	std::memcpy(&m_sendArrayMsg[8], &sendRequestDataObj.sHeader.sVersion.uRevision, 4);
	std::memcpy(&m_sendArrayMsg[12], &sendRequestDataObj.sHeader.uWatchdogCounter, 4);
	std::memcpy(&m_sendArrayMsg[16], &sendRequestDataObj.sHeader.uAreaNumber, 4);
	std::memcpy(&m_sendArrayMsg[20], &sendRequestDataObj.sCommand.eActualCommandVSComm, 4);
	std::memcpy(&m_sendArrayMsg[24], &sendRequestDataObj.sProductsSituation.eActualProductsSituationVSComm, 4);
	std::memcpy(&m_sendArrayMsg[28], &sendRequestDataObj.sProductsSituation.classId, 4);
}

bool UdpHandler::waitForMessageUsec(unsigned int timeoutUsec)
{
	double timeDeltaUsec = 0.0;
	double tick = (double)cv::getTickCount();
	while (m_receiver->m_bDataReceived == false && timeDeltaUsec <= (double)timeoutUsec)
	{
		timeDeltaUsec = ((double)cv::getTickCount() - tick) / cv::getTickFrequency() * 1000000;
	}

	if (timeDeltaUsec > static_cast<double>(timeoutUsec))
	{
		m_kLogger->error("UdpHandler: Timeout! Udp did not recieve any messages within timout.");
		return true;
	}
	return false;
}

namespace udpHander
{
std::shared_ptr<UdpHandler> setupUdpHandler(const fs::path& networksConfigPath,
                                            const std::shared_ptr<spdlog::logger> kLogger)
{
	nlohmann::json jsonFile = utils::loadJsonFile(networksConfigPath);
	std::shared_ptr<UdpHandler> udpPickConnectionHandler;
	// assign default values
	std::string visionSystemIP = "";
		unsigned int visionSystemPort;
		std::string twinCatIP = "";
		unsigned int twinCatPort;
		int areaNumber;
		bool udpEnabled = false;
	try
	{
		visionSystemIP = jsonFile.at("udp_configuration").at("udp_visionSystemIP").get<std::string>();
		visionSystemPort = jsonFile.at("udp_configuration").at("udp_visionSystemPort").get<int>();
		twinCatIP = jsonFile.at("udp_configuration").at("udp_twinCatIP").get<std::string>();
		twinCatPort = jsonFile.at("udp_configuration").at("udp_twinCatPort").get<int>();
		areaNumber = jsonFile.at("areaNumber").get<int>();
		udpEnabled = jsonFile.at("udp_configuration").at("udp_enabled").get<bool>();
	}
	catch (const std::exception& e)
	{
		kLogger->error("Could not read parameters from file: " + networksConfigPath.string());
		return udpPickConnectionHandler;
	}
	
	// for communication with TwinCat3 over udp for camera triggering
	// and rotational encoder, which enables us to pick from running belt
	// start connection to TwinCat which gives encoder, cameraTrigger etc.
	try
	{
		if (udpEnabled)
		{
			udpPickConnectionHandler =
			    std::make_shared<UdpHandler>(visionSystemIP, visionSystemPort, twinCatIP, twinCatPort, areaNumber, udpEnabled);
		}
		else
		{
			//  just a mock object when udp is disabled
			udpPickConnectionHandler =
			    std::make_shared<MockUdpHandler>(visionSystemIP, visionSystemPort, twinCatIP, twinCatPort, udpEnabled);
		}
	}
	catch (const std::exception& e)
	{
		kLogger->error(e.what());
		kLogger->error("Cannot create UdpHandler");
		return udpPickConnectionHandler;
	}
	udpPickConnectionHandler->m_configured = true;
	return udpPickConnectionHandler;
}
}  // namespace udpHander
