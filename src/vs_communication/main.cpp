/**
 * @copyright Copyright (c) 2018 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 *
 * @brief main communication process to control robot unit
 */
#include <getopt.h>
#include <math.h>

#include <chrono>  // for high_resolution_clock
#include <iostream>

#include "connectionHandler.h"
#include "logging/logging.h"
#include "vs.h"
#include "vs_c_queue.h"
#include "vs_c_version.h"
#include "vs_c_workItem.h"

#define WORKER (32)

void parseCommandLineInput(int argc,
                           char* argv[],
                           std::string& serverIpAdress,
                           const std::shared_ptr<spdlog::logger> kLogger)
{
	int c;
	const char* short_opt = "n:h";
	struct option long_opt[] = {
	    {"network", required_argument, NULL, 'n'}, {"help", no_argument, NULL, 'h'}, {NULL, 0, NULL, 0}};

	while ((c = getopt_long(argc, argv, short_opt, long_opt, NULL)) != -1)
	{
		switch (c)
		{
			case 'n': serverIpAdress = argv[2]; break;
			case 'h':
			case ':':
			case '?':
			default:
				kLogger->debug("Usage: {} [OPTIONS]", argv[0]);
				kLogger->debug("\t-n --network\t set network adress to bind socket");
				kLogger->debug("\t-h --help\tprint this help and exit");
				exit(0);
		}
	}
}

int main(int argc, char* argv[])
{
	static const std::shared_ptr<spdlog::logger> kLogger = logging::setupLogger("VsCommunication");
	kLogger->info("{} , {}", basename(argv[0]), VS_C_VERSION);

	std::string serverIpAddress;
	parseCommandLineInput(argc, argv, serverIpAddress, kLogger);

	// Create the queue and consumer (worker) threads
	vsQueue<WorkItem*> queue;

	for (int i = 0; i < WORKER; i++)
	{
		ConnectionHandler* handler = new ConnectionHandler(queue);
		if (!handler)
		{
			kLogger->error("Could not create ConnectionHandler {}", i);
			exit(1);
		}
		handler->start();  // every ConnectionHandler spawns an own thread in  which it handles incomming requests from
		                   // VMS and PS. It is a heap object, therefore it won't get destoyed after this line (scope)
	}
	// Create an acceptor then start listening for connections
	WorkItem* item;
	TCPAcceptor* connectionAcceptor;

	if (serverIpAddress.length() > 0)
	{
		connectionAcceptor = new TCPAcceptor(VS_C_SERVER_PORT, (char*)serverIpAddress.c_str());
		kLogger->debug("Listen on address: {} port: {}", serverIpAddress.c_str(), VS_C_SERVER_PORT);
	}
	else
	{
		connectionAcceptor = new TCPAcceptor(VS_C_SERVER_PORT, VS_C_DEFAULT_IP);
		kLogger->debug("Listen on address: {} port: {}", VS_C_DEFAULT_IP, VS_C_SERVER_PORT);
	}

	if (!connectionAcceptor)
	{
		kLogger->error("Could not create an connection acceptor");
		exit(1);
	}

	while (connectionAcceptor->start() != 0)  // in case jetson is booting faster than vms
	{
		kLogger->error("try to start a connection acceptor");
		sleep(1);
	}

	// Add a work item to the queue for each connection
	while (true)
	{
		TCPStream* connection = connectionAcceptor->accept();
		if (!connection)
		{
			kLogger->error("Could not accept a connection");
			continue;
		}
		item = new WorkItem(connection);
		if (!item)
		{
			kLogger->error("Could not create work item");
			continue;
		}
		queue.add(item);
	}

	return 0;
}
