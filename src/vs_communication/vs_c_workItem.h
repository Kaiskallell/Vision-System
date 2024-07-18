/*
 * Created on Wed Nov 06 2019
 *
 * Copyright (c) 2019 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 */

#ifndef WORK_ITEM_H
#define WORK_ITEM_H

#include "vs_c_queue.h"
#include "vs_c_tcpAcceptor.h"
#include "vs_c_thread.h"

// work item with tcp stream
class WorkItem
{
	TCPStream* m_stream;

  public:
	WorkItem(TCPStream* stream) : m_stream(stream) {}
	~WorkItem() { delete m_stream; }

	TCPStream* getStream() { return m_stream; }
};

#endif