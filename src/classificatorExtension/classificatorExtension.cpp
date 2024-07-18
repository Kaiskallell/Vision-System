/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 */

#include "classificatorExtension.h"

#include "cameraFactory/cameraFactory.hpp"
#include "miscellaneous.h"

namespace visionsystemsignals
{
extern bool shutdownProgam;  // defined in vs_signalHandler.h
}

ClassificationExtension::ClassificationExtension(const fs::path& cameraConfigPath,
                                                 const fs::path& visionSystemConfigPath,
                                                 const fs::path& networksConfigPath)
{
	m_dbFacade = std::make_shared<db::DbFacade>(visionSystemConfigPath);
	// get formatId
	nlohmann::json jsonFile = utils::loadJsonFile(networksConfigPath);
	try
	{
		m_areaNumber = jsonFile.at("areaNumber").get<int>();  // areaNumber: 7 or 8 for extensions, now is only area 7 can be used within vms when applying EEC on PS
		m_vms_enabled = jsonFile.at("vms_configuration").at("vms_enabled").get<bool>(); // classification is done within vms connection
	}
	catch (const std::exception& e)
	{
		m_dbFacade->writeErrorCodeToDB(VS_ERR_FORMAT_GENERALPARAMETERS_CLASSIFICATOR);
		m_kLogger->error("Could not read parameters from file: " + networksConfigPath.string());
		return;
	}
	
	m_camera = setupCamera(visionsystemsignals::shutdownProgam, cameraConfigPath, networksConfigPath, m_kLogger);
	if(!m_camera->m_configured)
	{
		m_dbFacade->writeErrorCodeToDB(VS_ERR_CAMERA_INIT_ERROR_CLASSIFICATOR);
		return;
	}
	size_t width = m_camera->getImgWidth();
	size_t height = m_camera->getImgHeight();
	m_kLogger->debug("Camera image width: {}", width);
	m_kLogger->debug("Camera image height: {}", height);
	m_detectionAreaMask =
	    misc::getDetectionAreaMask(networksConfigPath, m_camera->getSerialNumber(), cv::Size(width, height), m_kLogger);
	m_udpClassificatprExtensionConnectionHandler = udpHander::setupUdpHandler(networksConfigPath, m_kLogger);
	if(!m_udpClassificatprExtensionConnectionHandler->m_configured)
	{
		m_dbFacade->writeErrorCodeToDB(VS_ERR_UDPHANDLER_ERROR);
		return;
	}
	m_yolo = std::make_unique<yolo::Yolo>(networksConfigPath);
	if(!m_yolo->m_config.configured)
	{
		m_dbFacade->writeErrorCodeToDB(VS_ERR_DETECTOR_WRONGPARAMETERS_CLASSIFICATOR);
		return;
	}
	int formatId = std::stoi(networksConfigPath.parent_path().stem().string());
	db::setupDBObjectForDetection(
	    m_dbFacade, m_imprintMappingTable, visionSystemConfigPath, formatId, m_areaNumber, m_kLogger);
	m_kLogger->debug("end of ClassificationExtension constructor");
}

ClassificationExtension::~ClassificationExtension() {}

void ClassificationExtension::detectPosesWithNetworks(int& classId, eProductsSituationVSComm& prodSituation)
{
	// m_kLogger->debug("NETWORKS");
	double dNetwork = (double)cv::getTickCount();
	// m_kLogger->debug("m_frame.colorImage.size= ", m_frame.colorImage.size());
	// m_kLogger->debug("m_detectionAreaMask.size= ", m_detectionAreaMask.size());
	assert(!m_frame.colorImage.empty());
	try
	{
		// make area black which is not the conveyor belt
		cv::bitwise_and(m_frame.colorImage, m_detectionAreaMask, m_frame.colorImage);
	}
	catch (const std::exception& e)
	{
		m_kLogger->error(e.what());
		return;
	}

	// here the actual inferencing happens
	std::shared_ptr<yolo::RectsAndClasses> rectsAndClasses = m_yolo->run(m_frame.colorImage, m_resultImage);
	if (rectsAndClasses->empty())
	{
		// m_kLogger->debug("No Detection made by m_yoloDlcPipeline");
		if(m_udpClassificatprExtensionConnectionHandler->m_udpEnabled)
		{
			m_udpClassificatprExtensionConnectionHandler->sendTo(eCommandVSComm_UPDATEPRODUCTSSITUATION,
		                                   						 eProductsSituationVSComm_NOTAVAILABLE);
		}
		
		return;
	}

	if (rectsAndClasses->size() > 1)  // only one product should be visible in turning station
	{
		m_kLogger->warn("rectsAndClasses->size() > 1");
		return;
	}

	classId = rectsAndClasses->at(0).second;

	// m_kLogger->debug("Modul found {} products.", m_keyPointsAndClasses.keyPoints.size());
	// m_kLogger->debug("NETWORKS step took {} ms",
	//                 (((double)cv::getTickCount() - dNetwork) / cv::getTickFrequency() * 1000));
}

void ClassificationExtension::detectPosesWithNetworks(int& classId)
{
	// m_kLogger->debug("NETWORKS");
	double dNetwork = (double)cv::getTickCount();
	// m_kLogger->debug("m_frame.colorImage.size= ", m_frame.colorImage.size());
	// m_kLogger->debug("m_detectionAreaMask.size= ", m_detectionAreaMask.size());
	assert(!m_frame.colorImage.empty());
	try
	{
		// make area black which is not the conveyor belt
		cv::bitwise_and(m_frame.colorImage, m_detectionAreaMask, m_frame.colorImage);
	}
	catch (const std::exception& e)
	{
		m_kLogger->error(e.what());
		return;
	}

	// here the actual inferencing happens
	std::shared_ptr<yolo::RectsAndClasses> rectsAndClasses = m_yolo->run(m_frame.colorImage, m_resultImage);
	if (rectsAndClasses->empty())
	{
		m_kLogger->debug("No Detection made by m_yoloDlcPipeline");	
		return;
	}

	if (rectsAndClasses->size() > 1)  // only one product should be visible in turning station
	{
		m_kLogger->warn("rectsAndClasses->size() > 1");
		return;
	}

	classId = rectsAndClasses->at(0).second;

	// m_kLogger->debug("Modul found {} products.", m_keyPointsAndClasses.keyPoints.size());
	// m_kLogger->debug("NETWORKS step took {} ms",
	//                 (((double)cv::getTickCount() - dNetwork) / cv::getTickFrequency() * 1000));
}

bool ClassificationExtension::run()
{
	if(m_vms_enabled)
	{
		runVMS();
	}
	else if (m_udpClassificatprExtensionConnectionHandler->m_udpEnabled)
	{
		runTC3();
	}
	else
	{
		runDebug();
	}
	return false;
}

void ClassificationExtension::runTC3()
{
	m_kLogger->warn("ClassificationExtension with UDP will be no more used in the future");

	m_udpClassificatprExtensionConnectionHandler->checkConnectionToTC3();

	while (!visionsystemsignals::shutdownProgam && !m_dbFacade->getShutDownProcesses())
	{
		misc::waitKey1ms();
		TimeStamp timeStamp;
		bool frameUpdated = false;
		bool ret = m_udpClassificatprExtensionConnectionHandler->askTC3ToMakeTriggerSignalAndGetEncoderVal(
		    															m_frameNumber, m_rotEncoderValue, timeStamp, frameUpdated);  // tc3 triggers the camera			
		if (!ret || !frameUpdated)
		{
			continue;
		}

		misc::getFrame(m_frame, m_resultImage, m_camera);  // gets the frame (color+ depth) out of the camera
		if (m_frame.colorImage.empty())
		{
			m_dbFacade->writeErrorCodeToDB(VS_ERR_CAMERA_EMPTYIMAGE_CLASSIFICATOR);
			continue;
		}
		m_frame.timeStamp = timeStamp;
		eProductsSituationVSComm prodSituation = eProductsSituationVSComm::eProductsSituationVSComm_AVAILABLE;
				
		int classId = -1;
		detectPosesWithNetworks(classId, prodSituation);  // makes the inference of yolo
		if (classId < 0)                   // no  detections
		{
			// All detections lead to bad poses so we need to take a new frame
			continue;
		}
		classId += 1;  // classId offset like L&R, never 0
		// send classId to tc3

		m_udpClassificatprExtensionConnectionHandler->sendToWithClassId(eCommandVSComm_UPDATEPRODUCTSSITUATION, 
														  				eProductsSituationVSComm_AVAILABLE, classId);	
				
		misc::showImage("debug_result_classificatorExtension", m_resultImage);
	}
}

void ClassificationExtension::runVMS()
{
	if(m_areaNumber != 7)
	{
		m_kLogger->error("ClassificationExtension with VMS Can handle now only area 7");
		return;
	}
	while (!visionsystemsignals::shutdownProgam && !m_dbFacade->getShutDownProcesses())
	{
		misc::waitKey1ms();
		TimeStamp timeStamp;
		bool extReq = m_dbFacade->getExtReq(m_areaNumber);
		bool ret = extReq && !m_extReqOld;  // Request from vms to take an image				
		if (!ret)
		{
			m_extReqOld = extReq;
			continue;
		}
		misc::getFrame(m_frame, m_resultImage, m_camera);  // gets the frame (color+ depth) out of the camera
		if (m_frame.colorImage.empty())
		{
			m_dbFacade->writeErrorCodeToDB(VS_ERR_CAMERA_EMPTYIMAGE_CLASSIFICATOR);
			continue;
		}
		m_frame.timeStamp = timeStamp;		
		// This is only placeholders when communicating directly with VMS
		m_frameNumber += 1;
		std::vector<VSPose> posesInRobotCoordinates;
		VSPose poseInRobot;
		posesInRobotCoordinates.push_back(poseInRobot);
		int classId = -1;
		std::vector<int> classIds;
		detectPosesWithNetworks(classId);  // makes the inference of yolo
		if (classId < 0)                   // no  detections
		{
			// All detections lead to bad poses so we need to take a new frame
			continue;
		}	
		classId += 1;  // classId offset like L&R, never 0
		classIds.push_back(classId);
		db::writePosesToDB(m_dbFacade,
		                   posesInRobotCoordinates,
		                   classIds,
		                   m_frameNumber,
		                   m_rotEncoderValue,
		                   m_areaNumber,
		                   m_imprintMappingTable,
		                   m_kLogger);  // makes the poses accessible in vsCommunication				
		
		misc::showImage("debug_result_classificatorExtension", m_resultImage);
		m_extReqOld = extReq;
	}
}

void ClassificationExtension::runDebug()
{
	while (!visionsystemsignals::shutdownProgam && !m_dbFacade->getShutDownProcesses())
	{
		misc::waitKey1ms();
		TimeStamp timeStamp;
		misc::getFrame(m_frame, m_resultImage, m_camera);  // gets the frame (color+ depth) out of the camera
		if (m_frame.colorImage.empty())
		{
			m_dbFacade->writeErrorCodeToDB(VS_ERR_CAMERA_EMPTYIMAGE_CLASSIFICATOR);
			continue;
		}
		m_frame.timeStamp = timeStamp;		
		// This is only placeholders when communicating directly with VMS
		int classId = -1;
		detectPosesWithNetworks(classId);  // makes the inference of yolo		
		misc::showImage("debug_result_classificatorExtension", m_resultImage);
	}
}
