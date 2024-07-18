/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved

 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 *
 */

#include "placeConveyorDetection.h"

#include "cameraFactory/cameraFactory.hpp"
#include "miscellaneous/miscellaneous.h"

namespace visionsystemsignals
{
extern bool shutdownProgam;  // defined in vs_signalHandler.h
}

PlaceConvDetection::PlaceConvDetection(const fs::path& cameraConfigPath,
                                       const fs::path& visionSystemConfigPath,
                                       const fs::path& networksConfigPath)
{
	m_dbFacade = std::make_shared<db::DbFacade>(visionSystemConfigPath);
	// get formatId
	nlohmann::json jsonFile = utils::loadJsonFile(networksConfigPath);
	try
	{
		m_areaNumber = jsonFile.at("areaNumber").get<int>();  // areaNumber means  e.g. Transportstrecke 1 in VMS (camera 1)
	}
	catch (const std::exception& e)
	{
		m_dbFacade->writeErrorCodeToDB(VS_ERR_FORMAT_GENERALPARAMETERS_PLACE);
		m_kLogger->error("Could not read parameters from file: " + networksConfigPath.string());
		return;
	}
	
	try
	{
		m_yoloDlcPipeline = YoloDLC::setupYoloDlc(networksConfigPath, m_kLogger);
		if(!m_yoloDlcPipeline->m_moduleConfigured)
		{
			m_dbFacade->writeErrorCodeToDB(VS_ERR_DETECTOR_WRONGPARAMETERS_PLACE);
			return;
		}
	}
	catch (const std::exception& e)
	{
		m_dbFacade->writeErrorCodeToDB(VS_ERR_DETECTOR_WRONGPARAMETERS_PLACE);
		std::cerr << e.what() << '\n';
		return;
	}

	m_camera = setupCamera(visionsystemsignals::shutdownProgam, cameraConfigPath, networksConfigPath, m_kLogger);
	if(!m_camera->m_configured)
	{
		m_dbFacade->writeErrorCodeToDB(VS_ERR_CAMERA_INIT_ERROR_PLACE);
		return;
	}
	size_t width = m_camera->getImgWidth();
	size_t height = m_camera->getImgHeight();
	m_kLogger->debug("Camera image width: {}", width);
	m_kLogger->debug("Camera image height: {}", height);
	// only the conveyor belt should be visible to the neural network
	m_detectionAreaMask =
	    misc::getDetectionAreaMask(networksConfigPath, m_camera->getSerialNumber(), cv::Size(width, height), m_kLogger);

	// get udp_enabled, check if Vision System should communicate with TwinCat for triggering etc.
	m_udpPickConnectionHandler = udpHander::setupUdpHandler(networksConfigPath, m_kLogger);
	if(!m_udpPickConnectionHandler->m_configured)
	{
		m_dbFacade->writeErrorCodeToDB(VS_ERR_UDPHANDLER_ERROR);
		return;
	}
	m_poseEstimation = std::make_shared<PoseEstimation>(networksConfigPath);
	if(!m_poseEstimation->m_moduleConfigured)
	{
		m_dbFacade->writeErrorCodeToDB(VS_ERR_POSEESTIMATION_ERROR);
		return;
	}

	int formatId = std::stoi(networksConfigPath.parent_path().stem().string());
	db::setupDBObjectForDetection(
	    m_dbFacade, m_imprintMappingTable, visionSystemConfigPath, formatId, m_areaNumber, m_kLogger);
	m_moduleConfigured = true;
	m_kLogger->debug("end of placeConveyor constructor");
}

PlaceConvDetection::~PlaceConvDetection() {}

void PlaceConvDetection::detectPosesWithNetworks(std::vector<VSPose>& posesInRobotCoordinates,
                                                 std::vector<int>& classIds)
{
	double dNetwork = (double)cv::getTickCount();

	// m_kLogger->debug("m_frame.colorImage.size= ", m_frame.colorImage.size());
	// m_kLogger->debug("m_detectionAreaMask.size= ", m_detectionAreaMask.size());
	assert(!m_frame.colorImage.empty());

	cv::Mat networkInputImg;
	try
	{
		// make area black which is not the conveyor belt
		cv::bitwise_and(m_frame.colorImage, m_detectionAreaMask, networkInputImg);
	}
	catch (const std::exception& e)
	{
		m_kLogger->error(e.what());
	}

	// here the actual inferencing happens
	YoloDLCResults keyPointsAndClasses =
	    m_yoloDlcPipeline->run(networkInputImg, 5, YoloDLC::kAllClasses);
	if (keyPointsAndClasses.m_keyPoints.empty())
	{
		// m_kLogger->debug("No Detection made by m_yoloDlcPipeline");

		m_udpPickConnectionHandler->sendTo(eCommandVSComm_UPDATEPRODUCTSSITUATION,
		                                   eProductsSituationVSComm_NOTAVAILABLE);

		// m_kLogger->debug("NETWORKS step took {}ms",
		//                  (((double)cv::getTickCount() - dNetwork) / cv::getTickFrequency() * 1000));
		return;
	}

	// m_kLogger->debug("Modul found {} products.", keyPointsAndClasses.keyPoints.size());

	// m_kLogger->debug("NETWORKS step took {}ms",
	//                 (((double)cv::getTickCount() - dNetwork) / cv::getTickFrequency() * 1000));
	estimatePose(keyPointsAndClasses, posesInRobotCoordinates, classIds);

	m_udpPickConnectionHandler->sendTo(eCommandVSComm_UPDATEPRODUCTSSITUATION,
		                                   eProductsSituationVSComm_AVAILABLE);
}

void PlaceConvDetection::estimatePose(YoloDLCResults& keyPointsAndClasses,
                                      std::vector<VSPose>& posesInRobotCoordinates,
                                      std::vector<int>& classIds)
{
	double dPose = (double)cv::getTickCount();

	classIds.clear();                 // clear all old results
	posesInRobotCoordinates.clear();  // clear all old results

	for (int i = 0; i < keyPointsAndClasses.m_keyPoints.size(); ++i)  // iterating over all detected products
	{
		try
		{
			VSPose poseInRobot;
			// calculate pose and draw the poses to m_resultImage
			poseInRobot = m_poseEstimation->getPose(m_camera->vsGetCameraMatrix(),
			                                        m_camera->getExtrinsics(),
			                                        m_frame.depthImage,
			                                        keyPointsAndClasses.m_keyPoints.at(i),
			                                        keyPointsAndClasses.m_rects.at(i),
			                                        keyPointsAndClasses.m_classIds.at(i),
			                                        m_resultImage);
			if (keyPointsAndClasses.m_classIds.at(i) != CLASS_BAD_OBJECT_DIM)
			{
				posesInRobotCoordinates.push_back(poseInRobot);
				constexpr int kMagicOffsetForLAndR = 1;  // objects with classId==0 are not picked by L&R
				classIds.push_back(keyPointsAndClasses.m_classIds.at(i) + kMagicOffsetForLAndR);
				// m_kLogger->debug("product found with {} as class ID", keyPointsAndClasses.classIds[i]);
			}
		}
		catch (const std::exception& e)
		{
			m_kLogger->error(e.what());
			m_kLogger->error("Could not calculate pose");
		}
	}

	// in case someone wants to collect data at the robot for testing purposes
	// misc::saveDetectionsToDisk(posesInRobotCoordinates, m_frame, m_frameNumber, m_kLogger);

	// m_kLogger->debug("ESTIMATE_3D_POSE step took {}ms",
	//                 (((double)cv::getTickCount() - dPose) / cv::getTickFrequency() * 1000));
}

void PlaceConvDetection::run(std::shared_ptr<InterfaceDetTrk> interfaceDetTrk)
{
	std::vector<VSPose> posesInRobotCoordinates;
	std::vector<int> classIds;
	m_udpPickConnectionHandler->checkConnectionToTC3();

	// this is the loop which contols the detection program
	while (!visionsystemsignals::shutdownProgam && !m_dbFacade->getShutDownProcesses())
	{
		TimeStamp timeStamp;
		bool frameUpdated = false;
		misc::waitKey1ms();
		bool ret = m_udpPickConnectionHandler->askTC3ToMakeTriggerSignalAndGetEncoderVal(
		    m_frameNumber, m_rotEncoderValue, timeStamp, frameUpdated);  // tc3 triggers the camera and measures the
		                                                                 // conveyor belt encoder
		if (!ret)
		{
			continue;  // error with the communication to tc3
		}
		if (!frameUpdated)
		{
			interfaceDetTrk->lockInterface();
			// Update encoder value and timestamp
			cv::Mat emptyMat;
			std::vector<VSPose> emptyPoses;
			std::vector<int> emptyClasses;
			interfaceDetTrk->sendDataToTrk(timeStamp,
			                               emptyMat,
			                               emptyPoses,
			                               emptyClasses,
			                               m_frameNumber,
			                               m_rotEncoderValue,
			                               m_areaNumber,
			                               frameUpdated);  // send detected poses to tracking Modul
			interfaceDetTrk->unlockInterface();
			continue;
		}

		misc::getFrame(m_frame, m_resultImage, m_camera);  // gets the frame (color+ depth) out of the camera
		if (m_frame.colorImage.empty())
		{
			m_dbFacade->writeErrorCodeToDB(VS_ERR_CAMERA_EMPTYIMAGE_PLACE);
			continue;
		}

		misc::showImage("mask", m_detectionAreaMask);

		detectPosesWithNetworks(posesInRobotCoordinates, classIds);  // makes the inference of yolo and dlc
		
		misc::showImage("ObjectDetection", m_resultImage);

		// m_kLogger->debug("sendDataToTrk from detectionthread");

		interfaceDetTrk->lockInterface();
		interfaceDetTrk->sendDataToTrk(timeStamp,
		                               m_frame.colorImage,
		                               posesInRobotCoordinates,
		                               classIds,
		                               m_frameNumber,
		                               m_rotEncoderValue,
		                               m_areaNumber,
		                               frameUpdated);  // send detected poses to tracking Modul
		interfaceDetTrk->unlockInterface();
		posesInRobotCoordinates.clear();
		classIds.clear();
	}
}
