/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef CONNECTION_HANDLER_H
#define CONNECTION_HANDLER_H

#include "logging/logging.h"
#include "vs_c_communicationError.h"
#include "vs_c_queue.h"
#include "vs_c_workItem.h"
#include "vs_messaging.h"


// handle all incomming connections
class ConnectionHandler : public Thread
{
  private:
	vsQueue<WorkItem*>& m_queue;
	void vsSendNack(TCPStream* stream, eCommandIdentifier eComandID, int nSequenceNo, eResponseNACK_Errors eError);
	void vsToVmsSend(TCPStream* stream,
	                 const std::string& sMessageBuffer,
	                 const eCommandIdentifier eCommandID,
	                 const int nSequenceNo,
	                 const int nSize,
	                 const eResponseStatus status);
	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("ConnectionHandler");

  public:
	ConnectionHandler(vsQueue<WorkItem*>& queue);

	void* run();
};

#endif
