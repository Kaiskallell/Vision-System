/*
 * Created on Wed Oct 24 2018
 *
 * Copyright (c) 2018 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 */
#ifndef __VS_C_TCP_STREAM_H__
#define __VS_C_TCP_STREAM_H__

#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <string>

using namespace std;

class TCPStream
{
	int m_sd;
	string m_peerIP;
	int m_peerPort;

  public:
	friend class TCPAcceptor;
	friend class TCPConnector;

	~TCPStream();

	ssize_t send(const char* buffer, size_t len);
	ssize_t receive(char* buffer, size_t len, int timeout = 0);

	string getPeerIP();
	int getPeerPort();

	enum
	{
		connectionClosed = 0,
		connectionReset = -1,
		connectionTimedOut = -2
	};

  private:
	bool waitForReadEvent(int timeout);

	TCPStream(int sd, struct sockaddr_in* address);
	TCPStream();
	TCPStream(const TCPStream& stream);
};

#endif
