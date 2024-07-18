/**
 * @copyright Copyright (c) 2018 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 *
 * @brief main processing programm
 */

#include <signal.h>

#include <opencv2/opencv.hpp>

#include "detecting.h"
#include "logging/logging.h"
#include "miscellaneous/miscellaneous.h"
#include "projectPaths.h"
#include "spdlog/spdlog.h"
#include "vs.h"
#include "vs_signalHandler.h"

namespace fs = std::experimental::filesystem;

int main(int argc, char* argv[])
{
	// attach signal handler
	signal(SIGSEGV, signalHandler);
	signal(SIGINT, signalHandler);
	std::set_terminate(terminateHandler);

	// spdlog::set_level(spdlog::level::info);
	// spdlog::info("Welcome to spdlog!");
	// spdlog::error("Some error message with arg: {}", 1);
	const std::shared_ptr<spdlog::logger> logger = logging::setupLogger("main_picking");

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
		logger->error("e.g. : /home/cobot/Documents/git/VisionSystem/format/1/networksConfig.json");
		return -1;
	}

	detecting::start(cameraConfigPath, visionSystemConfigPath, networksConfigPath);

	return 0;
}
