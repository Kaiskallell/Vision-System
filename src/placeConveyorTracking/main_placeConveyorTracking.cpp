/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved

 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 *
 * @brief main program which starts and stops place conveyor tracking
 */

#include <opencv2/opencv.hpp>
#include <thread>

#include "logging/logging.h"
#include "miscellaneous/miscellaneous.h"
#include "objTracking.h"
#include "placeConveyorDetection.h"
#include "vs_signalHandler.h"

int main(int argc, char* argv[])
{
	// attach signal handler
	signal(SIGSEGV, signalHandler);
	signal(SIGINT, signalHandler);
	std::set_terminate(terminateHandler);
	const std::shared_ptr<spdlog::logger> logger = logging::setupLogger("main_placeConveyorTracking");

	fs::path defaultConfigRoot = utils::getProjectRootDir() / "config";
	fs::path cameraConfigPath = defaultConfigRoot / fs::path("pickCamera.json");
	fs::path visionSystemConfigPath = defaultConfigRoot / fs::path("visionSystemConfig.json");
	fs::path defaultFormatRoot = utils::getProjectRootDir() / "format";
	fs::path formatConfigPath = defaultFormatRoot / fs::path("format.json");
	fs::path defaultProgramsRoot = utils::getProjectRootDir() / "format";
	fs::path networksConfigPath = "";

	misc::parseCommandLineInput(
	    argc, argv, cameraConfigPath, visionSystemConfigPath, formatConfigPath, networksConfigPath);

	if (networksConfigPath.empty())
	{
		logger->error("networksConfigPath is empty. Please provide networksConfigPath via command line");
		logger->error("e.g. : /home/cobot/Documents/git/VisionSystem/format/1/networksConfigAreaPlace.json");
		return -1;
	}

	nlohmann::json jsonFile = utils::loadJsonFile(visionSystemConfigPath);
	misc::setVisualization(jsonFile.at("enableVisualization").get<bool>());

	PlaceConvDetection placeConvDetection(cameraConfigPath, visionSystemConfigPath, networksConfigPath);
	ObjTracking objTracking(networksConfigPath, visionSystemConfigPath, cameraConfigPath);
	if(objTracking.m_moduleConfigured && placeConvDetection.m_moduleConfigured)
	{
		std::shared_ptr<InterfaceDetTrk> interfaceDetTrk = std::make_shared<InterfaceDetTrk>();
		// start detection
		std::thread placeDetThread(&PlaceConvDetection::run, &placeConvDetection, interfaceDetTrk);
		// start tracking
		std::thread trackingThread(&ObjTracking::run, &objTracking, interfaceDetTrk);
		placeDetThread.join();
		trackingThread.join();
	}
	return 0;
}
