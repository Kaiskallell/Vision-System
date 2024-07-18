/*
 * Created on Wed Jun 12 2019
 *
 * Copyright (c) 2019 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 *
 */
#ifndef SIGNALHANDLER_H
#define SIGNALHANDLER_H

#include <execinfo.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>

#include <stdexcept>

#include "vs.h"

namespace visionsystemsignals
{
bool shutdownProgam = false;
}

static void printStacktrace()
{
	char* pLogPath;
	char sPath[64];

	pLogPath = getenv("~/.config/VisionSystem/logs/");
	if (pLogPath == NULL) exit(-1);
	snprintf(sPath, 63, "%scrash_report.txt", pLogPath);
	std::fprintf(stderr, "Print crash report: %s\n", sPath);

	int boomfile;
	const int buff_size = 50;

	boomfile = open(sPath, O_TRUNC | O_WRONLY, (S_IRUSR | S_IWUSR));
	if (boomfile != -1)
	{
		void* buffer[buff_size];
		int buff_used;

		buff_used = backtrace(buffer, buff_size);
		backtrace_symbols_fd(buffer, buff_used, boomfile);
		close(boomfile);
	}
	else
	{
		std::fprintf(stderr, "Error printing crash report: %s\n", sPath);
	}
}

/* To catch segmentation faults, you can register a signal handler that is called
 * when your programm receives a SIGSEGV signal (segmentation violation, code 11).
 * This is done through signal() from signal.h:
 */
void signalHandler(int sig)
{
	if (sig == SIGINT)
	{
		std::printf("Recieved SIGINT from ctrl+c input\n");
		visionsystemsignals::shutdownProgam = true;
	}
	else if (sig == SIGTERM)
	{
		std::printf("Recieved SIGTERM probably from systemd restart or systemd stop\n");
		visionsystemsignals::shutdownProgam = true;
	}
	else
	{
		std::fprintf(stderr, "Error: signal %d\n", sig);
		printStacktrace();
		std::abort();
	}
}

/* When an exception in a C++ program is not caught by try-catch, std::terminate()
 * is called before the program aborts. In fact, the default implementation simply calls abort(),
 * but you can do something meaningful beforehand. This is how you register a terminate handler
 * with std::set_terminate as defined by the exception header:
 */
void terminateHandler()
{
	std::exception_ptr exptr = std::current_exception();
	if (exptr)
	{
		// the only useful feature of std::exception_ptr is that it can be rethrown...
		try
		{
			std::rethrow_exception(exptr);
		}
		catch (std::exception& ex)
		{
			std::fprintf(stderr, "Terminated due to exception: %s\n", ex.what());
		}
		catch (...)
		{
			std::fprintf(stderr, "Terminated due to unknown exception\n");
		}
	}
	else
	{
		std::fprintf(stderr, "Terminated due to unknown reason :(\n");
	}
	printStacktrace();
	std::abort();
}

#endif  // SIGNALHANDLER_H
