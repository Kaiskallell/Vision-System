/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 *
 * @brief main of calibration
 *
 */

#include "calibration.h"
#include "projectPaths.h"
#include "vs_signalHandler.h"

void parseCommandLineInput(int argc,
                           char* argv[],
                           fs::path& cameraConfigPath,
                           fs::path& visionSystemConfigPath,
                           fs::path& networksConfigPath)
{
	// overwriting the default values with the command line arguments/values
	std::string keys = "{cameraConfigPath   |                 | camera configuration path}"   // default value ""
	                   "{visionSystemConfigPath  |      | vision system configuration path}"  // default value ""
	                   "{networksConfigPath  |      | format configuration path}"             // default value ""
	                   "{help   |      | show help message}";

	cv::CommandLineParser parser(argc, argv, keys);
	if (parser.has("help"))
	{
		parser.printMessage();
		exit(-1);
	}
	if (parser.has("cameraConfigPath"))
	{
		cameraConfigPath = parser.get<std::string>("cameraConfigPath");
	}
	if (parser.has("visionSystemConfigPath"))
	{
		visionSystemConfigPath = parser.get<std::string>("visionSystemConfigPath");
	}
	if (parser.has("networksConfigPath"))
	{
		networksConfigPath = parser.get<std::string>("networksConfigPath");
	}
}

int main(int argc, char* argv[])
{
	// attach signal handler
	signal(SIGSEGV, signalHandler);
	signal(SIGINT, signalHandler);
	std::set_terminate(terminateHandler);
	const std::shared_ptr<spdlog::logger> logger = logging::setupLogger("main_calibration");

	fs::path cameraConfigPath = utils::getProjectRootDir() / "config/pickCamera.json";
	fs::path visionSystemConfigPath = utils::getProjectRootDir() / "config/visionSystemConfig.json";
	fs::path formatConfigPath = utils::getProjectRootDir() / "format/format.json";
	nlohmann::json jsonFile = utils::loadJsonFile(formatConfigPath);
	int formatId = jsonFile.at("id").get<int>();  // e.g. 4 or 11
	fs::path networksConfigPath = "";

	parseCommandLineInput(argc, argv, cameraConfigPath, visionSystemConfigPath, networksConfigPath);

	if (networksConfigPath.empty())
	{
		logger->error("networksConfigPath is empty. Please provide networksConfigPath via command line");
		logger->error("e.g. : /home/cobot/Documents/git/VisionSystem/format/999/networksConfig.json");
		return -1;
	}

	Calibration calib(cameraConfigPath, visionSystemConfigPath, networksConfigPath);
	calib.start();
	return 0;
}
