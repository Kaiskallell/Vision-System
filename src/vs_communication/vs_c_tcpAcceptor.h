/*
 * Created on Wed Oct 24 2018
 *
 * Copyright (c) 2018 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 */

#ifndef __VS_C_TCPACCEPTOR_H__
#define __VS_C_TCPACCEPTOR_H__

#include <netinet/in.h>

#include <string>

#include "vs_c_tcpStream.h"

using namespace std;

class TCPAcceptor
{
	int m_lsd;
	int m_port;
	string m_address;
	bool m_listening;

  public:
	TCPAcceptor(int port, const char* address = "");
	~TCPAcceptor();

	int start();
	TCPStream* accept();

  private:
	TCPAcceptor() {}
};

#endif
