/*
 * Created on Wed Oct 24 2018
 *
 * Copyright (c) 2018 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 *
 */
#include "vs_c_tcpStream.h"

#include <arpa/inet.h>

#include <iostream>

TCPStream::TCPStream(int sd, struct sockaddr_in* address) : m_sd(sd)
{
	char ip[50];
	inet_ntop(PF_INET, (struct in_addr*)&(address->sin_addr.s_addr), ip, sizeof(ip) - 1);
	m_peerIP = ip;
	m_peerPort = ntohs(address->sin_port);
}

TCPStream::~TCPStream() { close(m_sd); }

ssize_t TCPStream::send(const char* buffer, size_t len) { return write(m_sd, buffer, len); }

ssize_t TCPStream::receive(char* buffer, size_t len, int timeout)
{
	if (timeout <= 0) return read(m_sd, buffer, len);

	if (waitForReadEvent(timeout) == true)
	{
		return read(m_sd, buffer, len);
	}
	return connectionTimedOut;
}

string TCPStream::getPeerIP() { return m_peerIP; }

int TCPStream::getPeerPort() { return m_peerPort; }

bool TCPStream::waitForReadEvent(int timeout)
{
	fd_set sdset;
	struct timeval tv;

	tv.tv_sec = timeout;
	tv.tv_usec = 0;
	FD_ZERO(&sdset);
	FD_SET(m_sd, &sdset);
	if (select(m_sd + 1, &sdset, NULL, NULL, &tv) > 0)
	{
		return true;
	}
	return false;
}
