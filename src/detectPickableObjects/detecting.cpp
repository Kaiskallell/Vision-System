#include "detecting.h"

#include "detectFromTray.h"
#include "ipaDetection.h"
#include "miscellaneous.h"
#include "pickconveyor.h"
#include "stechtest.h"
#include "dbFacade/dbFacade.h"
#include "vs_c_communicationError.h"


void detecting::start(const fs::path& cameraConfigPath,
                      const fs::path& visionSystemConfigPath,
                      const fs::path& networksConfigPath)
{
	static const std::shared_ptr<spdlog::logger> logger = logging::setupLogger("detecting");
	// handles data base interactions
	std::shared_ptr<db::DbFacade> dbFacade = std::make_shared<db::DbFacade>(visionSystemConfigPath);

	nlohmann::json jsonFile = utils::loadJsonFile(visionSystemConfigPath);
	misc::setVisualization(jsonFile.at("enableVisualization").get<bool>());
	// choose between tray and conveyor
	jsonFile = utils::loadJsonFile(networksConfigPath);
	bool stechtestActive = false; // assign default value
	try
	{
		stechtestActive = jsonFile.at("stechtestConfigs").at("active").get<bool>();
	}
	catch (const std::exception& e)
	{
		dbFacade->writeErrorCodeToDB(VS_ERR_FORMAT_GENERALSTECHTESTPARAMETERS);
		logger->error("Could not read parameters from file: " + networksConfigPath.string());
		return;
	}
	
	if(!stechtestActive)
	{
		std::string mode ="";
		try
		{
			mode = jsonFile.at("appConfigs").at("generalConfigs").at("pickMode").get<std::string>();
		}
		catch (const std::exception& e)
		{
			dbFacade->writeErrorCodeToDB(VS_ERR_FORMAT_GENERALPARAMETERS_PICK);
			logger->error("Could not read parameters from file: " + networksConfigPath.string());
			return;
		}
		logger->debug("start {}", mode);
		if (mode == "conveyor")
		{
			PickConveyor pickConveyor(cameraConfigPath, visionSystemConfigPath, networksConfigPath);
			if(pickConveyor.m_moduleConfigured)
			{
				// to deliver pickable products
				pickConveyor.run();
			}
			else
			{
				return;
			}
			
		}
		else if (mode == "tray")
		{
			DetectFromTray detectFromTray(cameraConfigPath, visionSystemConfigPath, networksConfigPath);
			if(detectFromTray.m_moduleConfigured)
			{
				detectFromTray.run();
			}
			else
			{
				return;
			}		
		}
#ifdef __aarch64__
		else if (mode == "ipaDetection")
		{
			IpaDetection ipaDetection(cameraConfigPath, visionSystemConfigPath, networksConfigPath);
			ipaDetection.run();
		}
#endif
		else
		{
			logger->debug("No valid pick mode! Choose 'tray' or 'conveyor' or 'ipaDetection'.");
		}
	}
	else
	{
		logger->debug("start {}", "StechTest");
		Stechtest stechtest(cameraConfigPath, visionSystemConfigPath, networksConfigPath);
		if(stechtest.m_moduleConfigured)
		{
			stechtest.run();
		}
		else
		{
			return;
		}
	}
}
