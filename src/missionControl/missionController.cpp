/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "missionController.h"

#include <boost/process.hpp>

#include "json.hpp"
#include "projectPaths.h"

namespace bp = boost::process;
namespace visionsystemsignals
{
extern bool shutdownProgam;
}

MissionController::MissionController()
{
	m_dbFacade = std::make_unique<db::DbFacade>(utils::getProjectRootDir() / "config/visionSystemConfig.json");
	m_dbFacade->flushDB();
	m_dbFacade->setFormatChanged(false);            // no inital format change
	m_dbFacade->setShutDownProcesses(false);  // not shutdown we want to start it
	m_formatConfigPath = utils::getProjectRootDir() / "format/format.json";

	// get formatId
	nlohmann::json jsonFile = utils::loadJsonFile(m_formatConfigPath);
	int formatId = jsonFile.at("id").get<int>();  // e.g. 4 or 11
	m_dbFacade->writeFormatToDB(formatId);

	fs::path networksConfigPathFolder = m_formatConfigPath.parent_path() / fs::path(std::to_string(formatId));

	constexpr int kCalibrationFormatId = 999;
	if (formatId == kCalibrationFormatId)
	{
		startCalibProcesses();
	}
	else
	{
		startDetectionProcesses(networksConfigPathFolder);
	}
}

MissionController::~MissionController()
{
	try
	{
		// close every process so that we can press ctrl+c and everything is shut down
		m_kLogger->debug("closing m_vsCommunicationProcess");
		m_vsCommunicationProcess.terminate();
		m_kLogger->debug("closing verify, detect and calibration processes");
		closingAllProcesses();
	}
	catch (const std::exception& e)
	{
		m_kLogger->error(e.what());
		m_kLogger->error("Cannot terminate all processes in destructor. You may have to kill them manually!");
	}
	m_dbFacade->setShutDownProcesses(
	    false);  // in case someone wants to start detectPickable as stand alone program for debugging etc.
}

void MissionController::startDetectionProcesses(const fs::path& networksConfigPathFolder)
{
	m_dbFacade->setShutDownProcesses(false);  // not shutdown we want to start it
	// start detectPickable
	fs::path detectPickablePath = utils::getProjectRootDir() / "build/bin/detectPickableObjects";
	fs::path placeTrackingPath = utils::getProjectRootDir() / "build/bin/placeConveyorTracking";
	fs::path classificatorExtensionPath = utils::getProjectRootDir() / "build/bin/classificatorExtension";

	// check how many detectPickable executables we need to launch
	for (auto& networksConfigPath : std::experimental::filesystem::directory_iterator(networksConfigPathFolder))
	{
		// check if it is a config file for detection or tracking (needs to contain 'Place' in its name e.g.
		// 'networksConfigPlaceArea.json')
		std::string placeString = "Place";
		std::string classificatorString = "ClassificatorExtension";

		bool isPlaceConfigFile = boost::algorithm::contains(networksConfigPath.path().string(), placeString);
		bool isClassificatorConfigFile =
		    boost::algorithm::contains(networksConfigPath.path().string(), classificatorString);
		if (isPlaceConfigFile)
		{
			m_kLogger->debug("starting placeConveyorTracking: {}", networksConfigPath.path());
			const std::string placeTrackingArgs = "-networksConfigPath=" + networksConfigPath.path().string();
			m_placeConveyorTrackingProcesses.emplace_back(
			    bp::child(placeTrackingPath.string(),
			              bp::args(placeTrackingArgs),
			              m_placeTrackingGroup));  // group will allow to terminates process and all subprocesses
		}
		else if (isClassificatorConfigFile)
		{
			m_kLogger->debug("starting classificator executable: {}", networksConfigPath.path());
			const std::string classificatorArgs = "-networksConfigPath=" + networksConfigPath.path().string();
			m_classificatorExtensionProcesses.emplace_back(bp::child(
			    classificatorExtensionPath.string(),
			    bp::args(classificatorArgs),
			    m_classificatorExtensionGroup));  // group will allow to terminates process and all subprocesses
		}
		else
		{
			m_kLogger->debug("starting detectPickable: {}", networksConfigPath.path());
			const std::string detectArgs = "-networksConfigPath=" + networksConfigPath.path().string();
			m_detectPickableProcesses.emplace_back(
			    bp::child(detectPickablePath.string(),
			              bp::args(detectArgs),
			              m_detectProcessGroup));  // group will allow to terminates process and all subprocessess
		}
	}

	startCommunicationProcess();
	startHttpServerProcess();
}

void MissionController::startCalibProcesses()
{
	// start Calibration
	m_kLogger->debug("starting calibration executables");
	sleep(4);  // TODO(aschaefer): Bug: if baumer camera is used we need to wait until it is really closed
	int formatId = 999;
	m_dbFacade->writeFormatToDB(formatId);  // needed in getProductionState for VMS otherwise sending error to VMS

	fs::path calibPath = utils::getProjectRootDir() / "build/bin/calibration";
	const std::string calibArgs = "";
	m_calibProcess = bp::child(calibPath.string(), bp::args(calibArgs), m_calibGroup);
	startCommunicationProcess();
}

void MissionController::startCommunicationProcess()
{
	// start vsCommunication
	if (!m_vsCommunicationProcess.running())  // avoid to spawn accidentally multiple VsCommunication processes
	{
		m_kLogger->debug("starting VsCommunication");
		fs::path commPath = utils::getProjectRootDir() / "build/bin/VsCommunication";
		std::vector<std::string> commArgs = {"-n", "192.168.52.120"};
		m_vsCommunicationProcess = bp::child(commPath.string(), bp::args(commArgs));
	}
}

void MissionController::startHttpServerProcess()
{
	// start vsCommunication
	if (!m_httpServerProcess.running())  // avoid to spawn accidentally multiple VsCommunication processes
	{
		m_kLogger->debug("starting httpServerProcess");
		fs::path pathToHttpExec = utils::getProjectRootDir() / "build/bin/vsHttpServer";
		std::vector<std::string> httpServerArgs = {"192.168.52.120", "55000"};
		m_httpServerProcess = bp::child(pathToHttpExec.string(), bp::args(httpServerArgs));
	}
}

void MissionController::closingAllProcesses()
{
	// closes all processes execpt VsCommunication
	constexpr unsigned int kDaemonClosingDurationInSeconds = 10;
	try
	{
		m_dbFacade->setShutDownProcesses(true);  // is needed to cleanly shutdown HDV and when using threads

		if (m_detectProcessGroup.valid())
		{
			m_kLogger->debug("waiting for m_detectProcessGroup termination");
			for (int i = 0; i < m_detectPickableProcesses.size(); ++i)
			{
				m_detectPickableProcesses.at(i).wait();
				m_dbFacade->deleteSerializedObjInfosFromDB();
			}
			m_kLogger->debug("m_detectProcessGroup finished");
		}

		if (m_placeTrackingGroup.valid())
		{
			m_kLogger->debug("waiting for m_placeTrackingGroup termination");
			for (int i = 0; i < m_placeConveyorTrackingProcesses.size(); ++i)
			{
				m_placeConveyorTrackingProcesses.at(i).wait();
				m_dbFacade->deleteSerializedObjInfosFromDB();
			}
			m_kLogger->debug("m_placeTrackingGroup finished");
		}

		if (m_classificatorExtensionGroup.valid())
		{
			m_kLogger->debug("waiting for m_classificatorExtensionGroup termination");
			for (int i = 0; i < m_classificatorExtensionProcesses.size(); ++i)
			{
				m_classificatorExtensionProcesses.at(i).wait();
				m_dbFacade->deleteSerializedObjInfosFromDB();
			}
			m_kLogger->debug("m_classificatorExtensionGroup finished");
		}

		if (m_calibGroup.valid())
		{
			m_kLogger->debug("terminate m_calibGroup");
			m_calibGroup.terminate();
			m_calibProcess.wait();
		}
	}
	catch (const std::exception& e)
	{
		m_kLogger->error(e.what());
		m_kLogger->error("Cannot terminate process group");
	}
}

void MissionController::start()
{
	// constantly checking if there is a format change and the closing all running processes and restart
	while (true)
	{
		if (visionsystemsignals::shutdownProgam)
		{
			break;  // exit while loop, so destructor is getting called
		}

		if (!m_dbFacade->checkIfFormatChanged())
		{
			sleep(1);  // dont use cpu to much
			continue;  // do nothing if format was not changed
		}
		m_dbFacade->setFormatChanged(false);  // reset it, so user can do an other format change
		m_kLogger->debug("changing format");

		closingAllProcesses();

		uint32_t formatId = m_dbFacade->getFormatId();
		constexpr int kCalibrationFormatId = 999;
		if (formatId == kCalibrationFormatId)
		{
			startCalibProcesses();
		}
		else
		{
			fs::path networksConfigPathFolder = m_formatConfigPath.parent_path() / fs::path(std::to_string(formatId));
			startDetectionProcesses(networksConfigPathFolder);
		}
	}
}
