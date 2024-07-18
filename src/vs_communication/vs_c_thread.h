/*
 * Created on Wed Oct 24 2018
 *
 * Copyright (c) 2018 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 */

#ifndef __VS_C_THREAD_H__
#define __VS_C_THREAD_H__

#include <pthread.h>

class Thread
{
  public:
	Thread();
	virtual ~Thread();

	int start();
	int join();
	int detach();
	pthread_t self();

	virtual void* run() = 0;

  private:
	pthread_t m_tid;
	int m_running;
	int m_detached;
};

#endif
