/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef PROJECT_PATH_H
#define PROJECT_PATH_H

#include <experimental/filesystem>
#include <fstream>

#include "json.hpp"

namespace fs = std::experimental::filesystem;

namespace utils
{
// get top folder of VisionSystem project
inline fs::path getProjectRootDir()
{
	fs::path executablePath = fs::canonical("/proc/self/exe");
	fs::path binPath = executablePath.parent_path();
	fs::path buildPath = binPath.parent_path();
	fs::path projectRootPath = buildPath.parent_path();
	return projectRootPath;
}

inline fs::path getHomeDir() { return fs::path(getenv("HOME")); }

inline nlohmann::json loadJsonFile(const fs::path& configFilePath)
{
	std::ifstream ifs(configFilePath.string());
	if (!ifs.is_open())
	{
		throw std::runtime_error("cannot open config file! " + configFilePath.string());
	}
	
	nlohmann::json jsonFile;
	try
	{
	        jsonFile = nlohmann::json::parse(ifs);
	}
	catch(const std::exception& e)
	{
		std::string errMsg = e.what();
		errMsg = errMsg + "\n cannot parse config file! " + configFilePath.string();
		throw std::runtime_error(errMsg);
	}

	return jsonFile;
}

inline int getCamIdxFromCameraConfig(const std::string& serialNumber, const fs::path& cameraConfigPath)
{
	nlohmann::json jsonFile = loadJsonFile(cameraConfigPath);
	int nCameras = jsonFile.at("camera").size();
	for (int idx = 0; idx < nCameras; ++idx)
	{
		std::string configFileSerialNumber = jsonFile.at("camera").at(idx).at("serialNumber");
		if (configFileSerialNumber == serialNumber)
		{
			return idx;
		}
	}
	throw std::runtime_error("No matching serialNumber found in cameraConfigPath: " + serialNumber);
}

inline int getCamIdxFromNetworksConfig(const std::string& serialNumber, const fs::path& networksConfigPath)
{
	nlohmann::json jsonFile = loadJsonFile(networksConfigPath);
	int nCameras = jsonFile.at("cameraConfigs").at("camera").size();
	for (int idx = 0; idx < nCameras; ++idx)
	{
		std::vector<std::string> configFileSerialNumbers =
		    jsonFile.at("cameraConfigs").at("camera").at(idx).at("serialNumbers");
		for (int j = 0; j < configFileSerialNumbers.size(); ++j)
		{
			if (configFileSerialNumbers.at(j) == serialNumber)
			{
				return idx;
			}
		}
	}
	throw std::runtime_error("No matching serialNumber found in networksConfigPath: " + serialNumber);
}

inline int cvtNetConfIdx2CamConfIdx(const int netConfIdx,
                                    const fs::path networkConfigPath,
                                    const fs::path cameraConfigPath)
{
	nlohmann::json jsonFile = utils::loadJsonFile(networkConfigPath);
	std::string serialNo = jsonFile.at("cameraConfigs").at("camera").at(netConfIdx).at("serialNumbers").at(0);
	int camConfigIdx = utils::getCamIdxFromCameraConfig(serialNo, cameraConfigPath);
	return camConfigIdx;
}

}  // namespace utils

#endif
