/**
 * @copyright Copyright (c) 2021 Gerhard Schubert GmbH - All Rights Reserved

 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 *
 * @brief main program which stats and stops all other processes of VisionSystem
 */

#include <signal.h>
#include <unistd.h>

#include <iostream>
#include <opencv2/opencv.hpp>

#include "missionController.h"
#include "vs.h"
#include "vs_signalHandler.h"

int main(int argc, char* argv[])
{
	// attach signal handler
	signal(SIGSEGV, signalHandler);
	signal(SIGINT, signalHandler);
	std::set_terminate(terminateHandler);

	// overwriting the default values with the command line arguments/values
	std::string keys = "{version   |                 | show version of visionSystem application}"  // default value ""
	                   "{help   |      | show help message}";

	cv::CommandLineParser parser(argc, argv, keys);
	if (parser.has("version"))
	{
		std::cout << "VS_VERSION = " << VS_VERSION << std::endl;
		std::cout << "VS_COMMUNICATION_ST_VERSION = " << VS_COMMUNICATION_ST_MAJOR_VERSION << "."
		          << VS_COMMUNICATION_ST_MINOR_VERSION << "." << VS_COMMUNICATION_ST_PATCH_VERSION << std::endl;
		std::cout << "VS_COMMUNICATION_PS_VERSION = " << VS_COMMUNICATION_PS_VERSION_MAJOR << "."
		          << VS_COMMUNICATION_PS_VERSION_MINOR << "." << VS_COMMUNICATION_PS_VERSION_PATCH << std::endl;
		std::cout << "VS_UDP_VERSION = " << VS_UDP_MAJOR_VERSION << "." << VS_UDP_MINOR_VERSION << "."
		          << VS_UDP_PATCH_VERSION << std::endl;

		exit(-1);
	}
	if (parser.has("help"))
	{
		parser.printMessage();
		exit(-1);
	}

	try
	{
		MissionController missionControl;
		missionControl.start();
	}
	catch (std::exception& e)
	{
		std::cout << e.what() << std::endl;
		return -1;
	}

	return 0;
}
