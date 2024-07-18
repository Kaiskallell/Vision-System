/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef MISSION_CONTROLLER_H
#define MISSION_CONTROLLER_H

#include <boost/process.hpp>

#include "dbFacade/dbFacade.h"
#include "spdlog/spdlog.h"

class MissionController
{
  public:
	MissionController();
	~MissionController();

	void start();

  private:
	void startDetectionProcesses(const fs::path& networksConfigPathFolder);
	void startCalibProcesses();
	void startCommunicationProcess();
	void startHttpServerProcess();

	/**
	 * @brief closes all processes execpt VsCommunication
	 */
	void closingAllProcesses();

	fs::path m_formatConfigPath;

	std::unique_ptr<db::DbFacade> m_dbFacade;
	std::vector<boost::process::child> m_detectPickableProcesses;
	std::vector<boost::process::child> m_placeConveyorTrackingProcesses;
	std::vector<boost::process::child> m_classificatorExtensionProcesses;

	boost::process::child m_verifyPlacedProcess;
	boost::process::child m_vsCommunicationProcess;
	boost::process::child m_calibProcess;
	boost::process::child m_httpServerProcess;

	boost::process::group m_detectProcessGroup;
	boost::process::group m_placeTrackingGroup;
	boost::process::group m_classificatorExtensionGroup;
	boost::process::group m_calibGroup;
	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("MissionController");
};

#endif
