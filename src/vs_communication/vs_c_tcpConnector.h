/*
 * Created on Wed Oct 24 2018
 *
 * Copyright (c) 2018 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 */

#ifndef __VS_C_TCPCONNECTOR_H__
#define __VS_C_TCPCONNECTOR_H__

#include <netinet/in.h>

#include "vs_c_tcpStream.h"

class TCPConnector
{
  public:
	TCPStream* connect(const char* server, int port);
	TCPStream* connect(const char* server, int port, int timeout);

  private:
	int resolveHostName(const char* host, struct in_addr* addr);
};

#endif
