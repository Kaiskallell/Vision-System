/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "cameraFactory.hpp"

#include <fstream>

#include "baumerCam.hpp"
#include "creStereoDepthCamDecorator.h"
#include "mockCamera.h"
#include "projectPaths.h"
#include "vs_gv5040FA.h"

// returns a camera obj specified in config file. Triggermode is needed to start a camera for hand eye
// calibration (trigger off) and detection (hardware trigger)
std::shared_ptr<VsCameraInterface> cameraFactory(const size_t networkConfigIdx,
                                                 const fs::path& cameraConfigPath,
                                                 const fs::path& networksConfigPath,
                                                 TriggerMode triggerMode,
                                                 bool maxResolution)
{
	// walk through all cameras
	std::shared_ptr<VsCameraInterface> camera;
	nlohmann::json jsonFile = utils::loadJsonFile(networksConfigPath);
	// assign default values
	std::string cameraType = "";
	std::vector<std::string> serialNumbers;
	try
	{
		cameraType = jsonFile.at("cameraConfigs").at("camera")[networkConfigIdx].at("type").get<std::string>();	
		serialNumbers =
	    jsonFile.at("cameraConfigs").at("camera")[networkConfigIdx].at("serialNumbers").get<std::vector<std::string>>();
	}
	catch (const std::exception& e)
	{
		throw std::runtime_error("Could not read parameters from file: " + networksConfigPath.string());
		return camera;
	}
	
	
	// mock camera
	if (cameraType == "MockCamera")
	{
		camera = std::make_shared<MockCamera>(cameraConfigPath, networksConfigPath, serialNumbers.at(0));
		camera->setType(cameraType);
	}
	else if (cameraType == "creStereoMock")
	{
		std::shared_ptr<VsCameraInterface> camera0 =
		    std::make_shared<MockCamera>(cameraConfigPath, networksConfigPath, serialNumbers.at(0));
		std::shared_ptr<VsCameraInterface> camera1 =
		    std::make_shared<MockCamera>(cameraConfigPath, networksConfigPath, serialNumbers.at(1));
		std::vector<std::shared_ptr<VsCameraInterface>> cameras;
		cameras.emplace_back(camera0);
		cameras.emplace_back(camera1);
		camera = std::make_shared<CreStereoCamDecorator>(networkConfigIdx, cameras, networksConfigPath, serialNumbers);
		camera->setType(cameraType);
	}
	else if (cameraType == "GV5040FA")
	{
		camera = std::make_shared<VsGV5040FA>(cameraConfigPath, networksConfigPath, serialNumbers.at(0), maxResolution);
		if(!camera->m_configured)
		{
			return camera;
		}
		camera->setType(cameraType);
	}
	else if (cameraType == "creStereoGV5040FA")
	{
		std::shared_ptr<VsCameraInterface> idsCamera0 =
		    std::make_shared<VsGV5040FA>(cameraConfigPath, networksConfigPath, serialNumbers.at(0), maxResolution);
		if(!idsCamera0->m_configured)
		{
			return camera;
		}
		std::shared_ptr<VsCameraInterface> idsCamera1 =
		    std::make_shared<VsGV5040FA>(cameraConfigPath, networksConfigPath, serialNumbers.at(1), maxResolution);
		if(!idsCamera1->m_configured)
		{
			return camera;
		}
		std::vector<std::shared_ptr<VsCameraInterface>> cameras;
		cameras.emplace_back(idsCamera0);
		cameras.emplace_back(idsCamera1);
		camera = std::make_shared<CreStereoCamDecorator>(networkConfigIdx, cameras, networksConfigPath, serialNumbers);
		if(!camera->m_configured)
		{
			return camera;
		}
		camera->setType(cameraType);
	}
	else if (cameraType == "BaumerCam")
	{
		camera = std::make_shared<BaumerCam>(cameraConfigPath, networksConfigPath, serialNumbers.at(0), maxResolution);
		if(!camera->m_configured)
		{
			return camera;
		}
		camera->setType(cameraType);
	}
	else if (cameraType == "creStereoBaumer")
	{
		std::shared_ptr<VsCameraInterface> baumerCamera0 =
		    std::make_shared<BaumerCam>(cameraConfigPath, networksConfigPath, serialNumbers.at(0), maxResolution);
		if(!baumerCamera0->m_configured)
		{
			return camera;
		}
		std::shared_ptr<VsCameraInterface> baumerCamera1 =
		    std::make_shared<BaumerCam>(cameraConfigPath, networksConfigPath, serialNumbers.at(1), maxResolution);
		if(!baumerCamera1->m_configured)
		{
			return camera;
		}
		std::vector<std::shared_ptr<VsCameraInterface>> cameras;
		cameras.emplace_back(baumerCamera0);
		cameras.emplace_back(baumerCamera1);
		camera = std::make_shared<CreStereoCamDecorator>(networkConfigIdx, cameras, networksConfigPath, serialNumbers);
		if(!camera->m_configured)
		{
			return camera;
		}
		camera->setType(cameraType);
	}
	else
	{
		throw std::runtime_error("Camera setup failed, unknown cameraType:" + cameraType
		                         + ".\n Valid cameraTypes are : Mock3D, GV5040FA");
		return camera;
	}

	bool ret = camera->init(triggerMode);

	if (ret == false)  // if error in e.g. HDV then shutdown program
	{
		throw std::runtime_error("camera initialization failed");
		return camera;
	}
	camera->m_configured = true;
	return camera;
}

std::shared_ptr<VsCameraInterface> setupCamera(bool& shutDownReturnVar,
                                               const fs::path& cameraConfigPath,
                                               const fs::path& networksConfigPath,
                                               const std::shared_ptr<spdlog::logger> kLogger)
{
	std::shared_ptr<VsCameraInterface> camera;
	size_t camIdx = 0;
	nlohmann::json jsonFile = utils::loadJsonFile(networksConfigPath);

	// assign default value
	TriggerMode triggerMode = TriggerMode::TriggerOff;
	try
	{
		triggerMode =
			jsonFile.at("cameraConfigs").at("camera")[camIdx].at("triggerMode").get<std::string>() == "Off"
				? TriggerMode::TriggerOff
				: TriggerMode::TriggerHardware;
	}
	catch (const std::exception& e)
	{
		kLogger->error("Could not read parameters from file: " + networksConfigPath.string());
		return camera;
	}
		
	try
	{
		// getting camera specified in the config file
		camera = cameraFactory(camIdx, cameraConfigPath, networksConfigPath, triggerMode);
		if(!camera->m_configured)
		{
			return camera;
		}
		kLogger->debug("Using camera: {}", camera->getType());
	}
	catch (const std::exception& e)
	{
		kLogger->error("Could not set up all cameras!");
		kLogger->error(e.what());
		shutDownReturnVar = true;
		sleep(5);  // Give time for user to read that error message
		return camera;
	}
	return camera;
}
