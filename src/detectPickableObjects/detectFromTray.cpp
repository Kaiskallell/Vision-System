/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "detectFromTray.h"

// cameras
#include <unistd.h>

#include <algorithm>  // std::min
#include <experimental/filesystem>

#include "cameraFactory/cameraFactory.hpp"
#include "miscellaneous.h"
#include "mockUdpHandler.hpp"
#include "projectPaths.h"

namespace visionsystemsignals
{
extern bool shutdownProgam;  // defined in vs_signalHandler.h
}

DetectFromTray::DetectFromTray(const fs::path& cameraConfigPath,
                               const fs::path& visionSystemConfigPath,
                               const fs::path& networksConfigPath)
{
	m_dbFacade = std::make_shared<db::DbFacade>(visionSystemConfigPath);
	// get formatId
	nlohmann::json jsonFile = utils::loadJsonFile(networksConfigPath);
	std::map<int, float> classToDLCIndexMapping;
	try
	{
		m_areaNumber = jsonFile.at("areaNumber").get<int>();  // areaNumber means  e.g. Transportstrecke 1 in VMS (camera 1)
		m_maxAmountOfProductsPerImg = jsonFile.at("appConfigs").at("generalConfigs").at("maxAmountOfProductsPerImg").get<int>();
		m_constZEnable = jsonFile.at("appConfigs").at("yolo").at("detectionConfigs").at("desiredClassesConstZ").at("enable").get<bool>();
	
		for (auto it = jsonFile.at("appConfigs").at("yolo").at("detectionConfigs").at("desiredClassesConstZ").at("classToValuesInMeterMapping").begin();
			it != jsonFile.at("appConfigs").at("yolo").at("detectionConfigs").at("desiredClassesConstZ").at("classToValuesInMeterMapping").end();
			++it)
		{
			classToDLCIndexMapping[std::stoi(it.key())] = it.value();
		}	
	}
	catch (const std::exception& e)
	{
		m_dbFacade->writeErrorCodeToDB(VS_ERR_FORMAT_GENERALPARAMETERS_PICK);
		m_kLogger->error("Could not read parameters from file: " + networksConfigPath.string());
		return;
	}
	
	m_constZValue = classToDLCIndexMapping.begin()->second;
	// assign default values
	cv::Point p0TrayRect;
	cv::Point p1TrayRect;
	cv::Point p2TrayRect;
	cv::Point p3TrayRect;
	try
	{
		p0TrayRect.x = jsonFile.at("appConfigs").at("generalConfigs").at("trayArea").at("u0").get<int>();
		p0TrayRect.y = jsonFile.at("appConfigs").at("generalConfigs").at("trayArea").at("v0").get<int>();
		p1TrayRect.x = jsonFile.at("appConfigs").at("generalConfigs").at("trayArea").at("u1").get<int>();
		p1TrayRect.y = jsonFile.at("appConfigs").at("generalConfigs").at("trayArea").at("v1").get<int>();
		p2TrayRect.x = jsonFile.at("appConfigs").at("generalConfigs").at("trayArea").at("u2").get<int>();
		p2TrayRect.y = jsonFile.at("appConfigs").at("generalConfigs").at("trayArea").at("v2").get<int>();
		p3TrayRect.x = jsonFile.at("appConfigs").at("generalConfigs").at("trayArea").at("u3").get<int>();
		p3TrayRect.y = jsonFile.at("appConfigs").at("generalConfigs").at("trayArea").at("v3").get<int>();
	}
	catch (const std::exception& e)
	{
		m_dbFacade->writeErrorCodeToDB(VS_ERR_FORMAT_GENERALPARAMETERS_PICK);
		m_kLogger->error("Could not read parameters from file: " + networksConfigPath.string());
		return;
	}
	
	m_firstTrayPolygon.emplace_back(p0TrayRect);
	m_firstTrayPolygon.emplace_back(p1TrayRect);
	m_firstTrayPolygon.emplace_back(p2TrayRect);
	m_firstTrayPolygon.emplace_back(p3TrayRect);

	m_camera = setupCamera(visionsystemsignals::shutdownProgam, cameraConfigPath, networksConfigPath, m_kLogger);
	if(!m_camera->m_configured)
	{
		m_dbFacade->writeErrorCodeToDB(VS_ERR_CAMERA_INIT_ERROR_PICK);
		return;
	}
	size_t width = m_camera->getImgWidth();
	size_t height = m_camera->getImgHeight();
	m_kLogger->debug("Camera image width: {}", width);
	m_kLogger->debug("Camera image height: {}", height);
	// only the conveyor belt should be visible to the neural network
	m_detectionAreaMask =
	    misc::getDetectionAreaMask(networksConfigPath, m_camera->getSerialNumber(), cv::Size(width, height), m_kLogger);
	m_udpPickConnectionHandler = udpHander::setupUdpHandler(networksConfigPath, m_kLogger);
	if(!m_udpPickConnectionHandler->m_configured)
	{
		m_dbFacade->writeErrorCodeToDB(VS_ERR_UDPHANDLER_ERROR);
		return;
	}
	m_yolo = std::make_unique<yolo::Yolo>(networksConfigPath);
	if(!m_yolo->m_config.configured)
	{
		m_dbFacade->writeErrorCodeToDB(VS_ERR_DETECTOR_WRONGPARAMETERS_PICK);
		return;
	}
	m_poseEstimation = std::make_shared<PoseEstimation>();
	if(!m_poseEstimation->m_moduleConfigured)
	{
		m_dbFacade->writeErrorCodeToDB(VS_ERR_POSEESTIMATION_ERROR);
		return;
	}
	int formatId = std::stoi(networksConfigPath.parent_path().stem().string());
	db::setupDBObjectForDetection(
	    m_dbFacade, m_imprintMappingTable, visionSystemConfigPath, formatId, m_areaNumber, m_kLogger);
	m_moduleConfigured = true;
	m_kLogger->debug("end of detectFromTray constructor");
}

DetectFromTray::~DetectFromTray() { cv::destroyAllWindows(); }

void DetectFromTray::detectPosesWithNetworks(std::vector<VSPose>& posesInRobotCoordinates,
                                             std::vector<int>& classIds,
                                             eProductsSituationVSComm& prodSituation)
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

		m_udpPickConnectionHandler->sendTo(eCommandVSComm_UPDATEPRODUCTSSITUATION,
		                                   eProductsSituationVSComm_NOTAVAILABLE);
		return;
	}

	// filter according to first tray
	yolo::RectsAndClasses productsInFirstTray;
	yolo::RectsAndClasses productsOutsideFirstTray;
	for (int i = 0; i < rectsAndClasses->size(); ++i)
	{
		int cx = rectsAndClasses->at(i).first.center.x;
		int cy = rectsAndClasses->at(i).first.center.y;
		cv::Point2f centerPoint(cx, cy);
		bool measureDistance = false;
		double pointInPolygon = cv::pointPolygonTest(m_firstTrayPolygon, centerPoint, measureDistance);
		if (pointInPolygon > 0.0)  // positiv means inside
		{
			productsInFirstTray.emplace_back(rectsAndClasses->at(i));
		}
		else
		{
			productsOutsideFirstTray.emplace_back(rectsAndClasses->at(i));
		}
	}
	//  check if bad product is present
	if (std::any_of(productsInFirstTray.begin(),
	                productsInFirstTray.end(),
	                [](std::pair<cv::RotatedRect, yolo::ClassId> product) { return product.second != 0; }))
	{
		rectsAndClasses = {};
		m_kLogger->debug("Bad product detected. Skipping this tray.");
		prodSituation = eProductsSituationVSComm_BADPRODUCT;
		m_udpPickConnectionHandler->sendTo(eCommandVSComm_UPDATEPRODUCTSSITUATION, prodSituation);
		return;
	}
	if (productsInFirstTray.empty())
	{
		prodSituation = eProductsSituationVSComm_AVAILABLE_SECONDTRAY;
		rectsAndClasses = std::make_shared<yolo::RectsAndClasses>(productsOutsideFirstTray);
	}
	else
	{
		prodSituation = eProductsSituationVSComm_AVAILABLE;
		rectsAndClasses = std::make_shared<yolo::RectsAndClasses>(productsInFirstTray);
	}

	if (rectsAndClasses->size() > m_maxAmountOfProductsPerImg)
	{
		rectsAndClasses->resize(m_maxAmountOfProductsPerImg);  // avoid too many products sent to vms
	}

	// m_kLogger->debug("Modul found {} products.", m_keyPointsAndClasses.keyPoints.size());
	// m_kLogger->debug("NETWORKS step took {} ms",
	//                 (((double)cv::getTickCount() - dNetwork) / cv::getTickFrequency() * 1000));

	estimatePoses(rectsAndClasses, posesInRobotCoordinates, classIds);
}

void DetectFromTray::estimatePoses(const std::shared_ptr<yolo::RectsAndClasses> rectsAndClasses,
                                   std::vector<VSPose>& posesInRobotCoordinates,
                                   std::vector<int>& classIds)
{
	// m_kLogger->debug("ESTIMATE_3D_POSE");
	double dPose = (double)cv::getTickCount();

	classIds.clear();                 // clear all old results
	posesInRobotCoordinates.clear();  // clear all old results

	for (int i = 0; i < rectsAndClasses->size(); ++i)
	{
		if (!m_constZEnable)
		{
			m_kLogger->error(
			    "m_constZEnable is 'false' but detectFromTray needs constZ to work. Please enable 'constZ' "
			    "in config file.");
			break;
		}
		VSPose poseInRobot = m_poseEstimation->getPoseTray(m_camera->vsGetCameraMatrix(),
		                                                   m_camera->getExtrinsics(),
		                                                   rectsAndClasses->at(i).first.boundingRect(),
		                                                   m_constZValue);
		posesInRobotCoordinates.push_back(poseInRobot);
		constexpr int kMagicOffsetForLAndR = 1;  // objects with classId==0 are not picked by L&R
		classIds.push_back(rectsAndClasses->at(i).second + kMagicOffsetForLAndR);
	}

	// in case someone wants to collect data at the robot for testing purposes
	// misc::saveDetectionsToDisk(posesInRobotCoordinates, m_frame, m_frameNumber);

	// m_kLogger->debug("ESTIMATE_3D_POSE step took {}ms",
	//                (((double)cv::getTickCount() - dPose) / cv::getTickFrequency() * 1000));
}

void DetectFromTray::run()
{
	std::vector<VSPose> posesInRobotCoordinates;
	std::vector<int> classIds;

	// check if udp connection to TC3 is still working.
	m_udpPickConnectionHandler->checkConnectionToTC3();

	// this is the loop which contols the detection program
	while (!visionsystemsignals::shutdownProgam && !m_dbFacade->getShutDownProcesses())
	{
		misc::waitKey1ms();
		TimeStamp timeStamp;
		bool frameUpdated = false;
		bool ret = m_udpPickConnectionHandler->askTC3ToMakeTriggerSignalAndGetEncoderVal(
		    m_frameNumber, m_rotEncoderValue, timeStamp, frameUpdated);  // tc3 triggers the camera and measures the
		                                                                 // conveyor belt encoder
		if (!ret || !frameUpdated)
		{
			continue;
		}

		misc::getFrame(m_frame, m_resultImage, m_camera);  // gets the frame (color+ depth) out of the camera
		if (m_frame.colorImage.empty())
		{
			m_dbFacade->writeErrorCodeToDB(VS_ERR_CAMERA_EMPTYIMAGE_PICK);
			continue;
		}
		misc::showImage("mask", m_detectionAreaMask);
		m_frame.timeStamp = timeStamp;

		eProductsSituationVSComm prodSituation = eProductsSituationVSComm::eProductsSituationVSComm_AVAILABLE;
		detectPosesWithNetworks(
		    posesInRobotCoordinates, classIds, prodSituation);  // makes the inference of yolo and dlc
		if (posesInRobotCoordinates.size() == 0)                // no pose could be calculated with the given detections
		{
			// All detections lead to bad poses so we need to take a new frame
			continue;
		}

		db::writePosesToDB(m_dbFacade,
		                   posesInRobotCoordinates,
		                   classIds,
		                   m_frameNumber,
		                   m_rotEncoderValue,
		                   m_areaNumber,
		                   m_imprintMappingTable,
		                   m_kLogger);  // makes the poses accessible in vsCommunication
		m_udpPickConnectionHandler->sendTo(eCommandVSComm_UPDATEPRODUCTSSITUATION, eProductsSituationVSComm_AVAILABLE);

		// after we wrote all pose informations to the database we can delete all the data which lead to the pose
		m_camera->vsClearFrame();
		m_frame.colorImage.release();
		m_frame.depthImage.release();
		posesInRobotCoordinates.clear();
		classIds.clear();
		misc::showImage("debug_result", m_resultImage);
	}
}
