
/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef MOCKUDPHANDLER_H
#define MOCKUDPHANDLER_H

#include "udphandler.h"

// this class does nothing and is just a mock object when udp is disabled
class MockUdpHandler : public UdpHandler
{
  private:
	TimeStamp m_timeStamp;
	uint32_t m_rotEncoderVal;

  public:
	explicit MockUdpHandler(const std::string& listeningIpAdress,
	                        const size_t listeningUdpPort,
	                        std::string& sendingIpAdress,
	                        const size_t sendingUdpPort,
							const bool udpEnabled)
	{
		m_udpEnabled = udpEnabled;
		m_kLogger->debug("Using MockUdpHandler. udp is disabled. You can change it in config file.");
	}
	~MockUdpHandler() = default;

	void startRecieverThread() override{};
	bool sendTo(eCommandVSComm eCommandVSComm, eProductsSituationVSComm eProductsSituationVSComm) override
	{
		return false;  // no timeout occured
	};
	void checkConnectionToTC3() override{};
	bool askTC3ToMakeTriggerSignalAndGetEncoderVal(uint64_t& imageNumber,
	                                               uint32_t& rotEncoderVal,
	                                               TimeStamp& timeStamp,
	                                               bool& frameUpdated) override
	{
		m_kLogger->warn("No rotEncoderVal is used because you are using mockUdp");
		m_timeStamp.increment1ms();
		timeStamp = m_timeStamp;
		frameUpdated = true;
		return true;
	};

	bool sendToWithClassId(eCommandVSComm eCommandVSComm,
	                       eProductsSituationVSComm eProductsSituationVSComm,
	                       const uint32_t classId)
	{
		return false;  // no time out for response
	}
};

#endif  // MOCKUDPHANDLER_H
