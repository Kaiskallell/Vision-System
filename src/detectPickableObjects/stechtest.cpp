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
#include "stechtest.h"

namespace visionsystemsignals
{
extern bool shutdownProgam;  // defined in vs_signalHandler.h
}

Stechtest::Stechtest(const fs::path& cameraConfigPath,
                     const fs::path& visionSystemConfigPath,
                     const fs::path& networksConfigPath)
{
	m_dbFacade = std::make_shared<db::DbFacade>(visionSystemConfigPath);
	// get formatId
	nlohmann::json jsonFile = utils::loadJsonFile(networksConfigPath);
	try
	{
		m_areaNumber = jsonFile.at("areaNumber").get<int>();  // areaNumber means  e.g. Transportstrecke 1 in VMS (camera 1)
		m_constZValue = jsonFile.at("stechtestConfigs").at("distanceToStechBoard").get<float>();
		m_stechtestConfigs.m_filterByArea = jsonFile.at("stechtestConfigs").at("filterByArea").get<bool>();
		m_stechtestConfigs.m_minArea = jsonFile.at("stechtestConfigs").at("minArea").get<float>();
		m_stechtestConfigs.m_filterByCircularity = jsonFile.at("stechtestConfigs").at("filterByCircularity").get<bool>();
		m_stechtestConfigs.m_minCircularity = jsonFile.at("stechtestConfigs").at("minCircularity").get<float>();
		m_stechtestConfigs.m_filterByConvexity = jsonFile.at("stechtestConfigs").at("filterByConvexity").get<bool>();
		m_stechtestConfigs.m_minConvexity = jsonFile.at("stechtestConfigs").at("minConvexity").get<float>();
		m_stechtestConfigs.m_filterByInertia = jsonFile.at("stechtestConfigs").at("filterByInertia").get<bool>();
		m_stechtestConfigs.m_minInertiaRatio = jsonFile.at("stechtestConfigs").at("minInertiaRatio").get<float>();
	}
	catch (const std::exception& e)
	{
		m_dbFacade->writeErrorCodeToDB(VS_ERR_FORMAT_GENERALSTECHTESTPARAMETERS);
		m_kLogger->error("Could not read parameters from file: " + networksConfigPath.string());
		return;
	}
	m_maxAmountOfProductsPerImg = 50;

	

	m_camera = setupCamera(visionsystemsignals::shutdownProgam, cameraConfigPath, networksConfigPath, m_kLogger);
	if(!m_camera->m_configured)
	{
		m_dbFacade->writeErrorCodeToDB(VS_ERR_CAMERA_INIT_ERROR_STECHTEST);
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
	m_kLogger->debug("end of stechtest constructor");
}

Stechtest::~Stechtest() { cv::destroyAllWindows(); }

std::shared_ptr<yolo::RectsAndClasses> Stechtest::myBlobDetector(const cv::Mat& image, cv::Mat& debugImg)
{
	// Set our filtering parameters
	// Initialize parameter setting using cv2.SimpleBlobDetector
	cv::SimpleBlobDetector::Params params;

	// Set Area filtering parameters
	params.filterByArea = m_stechtestConfigs.m_filterByArea;
	params.minArea = m_stechtestConfigs.m_minArea;

	// Set Circularity filtering parameters
	params.filterByCircularity = m_stechtestConfigs.m_filterByCircularity;
	params.minCircularity = m_stechtestConfigs.m_minCircularity;  // 0.78 is rectangle

	// Set Convexity filtering parameters
	params.filterByConvexity = m_stechtestConfigs.m_filterByConvexity;
	params.minConvexity = m_stechtestConfigs.m_minConvexity;

	// Set inertia filtering parameters
	params.filterByInertia = m_stechtestConfigs.m_filterByInertia;
	params.minInertiaRatio = m_stechtestConfigs.m_minInertiaRatio;

	// Create a detector with the parameters
	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

	// Detect blobs
	std::vector<cv::KeyPoint> keypoints;
	detector->detect(image, keypoints);

	// Draw blobs on our image as red circles
	cv::drawKeypoints(image, keypoints, debugImg, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

	std::shared_ptr<yolo::RectsAndClasses> rectsAndClasses = std::make_shared<yolo::RectsAndClasses>();
	for (const auto& keypoint : keypoints)
	{
		rectsAndClasses->emplace_back(std::make_pair(
		    cv::RotatedRect(cv::Point(keypoint.pt.x, keypoint.pt.y), cv::Size(keypoint.size, keypoint.size), 0.0F), 0));
		cv::circle(debugImg, keypoint.pt, 1, cv::Scalar(255, 0, 0), -1);
	}

	// Display number of blobs
	const std::string text = "Number of Circular Blobs: " + std::to_string(keypoints.size());
	cv::putText(debugImg, text, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 100, 255), 2);

	return rectsAndClasses;
}

void Stechtest::detectPosesWithNetworks(std::vector<VSPose>& posesInRobotCoordinates,
                                        std::vector<int>& classIds,
                                        eProductsSituationVSComm& prodSituation)
{
	m_kLogger->debug("NETWORKS");
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
	std::shared_ptr<yolo::RectsAndClasses> rectsAndClasses = myBlobDetector(m_frame.colorImage, m_resultImage);
	if (rectsAndClasses->empty())
	{
		m_kLogger->debug("No Detection made by m_yoloDlcPipeline");

		m_udpPickConnectionHandler->sendTo(eCommandVSComm_UPDATEPRODUCTSSITUATION,
		                                   eProductsSituationVSComm_NOTAVAILABLE);
		return;
	}

	// m_kLogger->debug("Modul found {} products.", m_keyPointsAndClasses.keyPoints.size());
	m_kLogger->debug("NETWORKS step took {} ms",
	                 (((double)cv::getTickCount() - dNetwork) / cv::getTickFrequency() * 1000));

	estimatePoses(rectsAndClasses, posesInRobotCoordinates, classIds);
}

void Stechtest::estimatePoses(const std::shared_ptr<yolo::RectsAndClasses> rectsAndClasses,
                              std::vector<VSPose>& posesInRobotCoordinates,
                              std::vector<int>& classIds)
{
	m_kLogger->debug("ESTIMATE_3D_POSE");
	double dPose = (double)cv::getTickCount();

	classIds.clear();                 // clear all old results
	posesInRobotCoordinates.clear();  // clear all old results

	for (int i = 0; i < rectsAndClasses->size(); ++i)
	{
		VSPose poseInRobot = m_poseEstimation->getPoseTray(m_camera->vsGetCameraMatrix(),
		                                                   m_camera->getExtrinsics(),
		                                                   rectsAndClasses->at(i).first.boundingRect(),
		                                                   m_constZValue);
		posesInRobotCoordinates.push_back(poseInRobot);
		constexpr int kMagicOffsetForLAndR = 1;  // objects with classId==0 are not picked by L&R
		classIds.push_back(rectsAndClasses->at(i).second + kMagicOffsetForLAndR);
	}

	misc::stechtestPoseSelection(m_stechtestPoseIdx, posesInRobotCoordinates, classIds);

	// in case someone wants to collect data at the robot for testing purposes
	// misc::saveDetectionsToDisk(posesInRobotCoordinates, m_frame, m_frameNumber);

	// m_kLogger->debug("ESTIMATE_3D_POSE step took {}ms",
	//                (((double)cv::getTickCount() - dPose) / cv::getTickFrequency() * 1000));
}

void Stechtest::run()
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
			m_dbFacade->writeErrorCodeToDB(VS_ERR_CAMERA_EMPTYIMAGE_STECHTEST);
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
		misc::showImage("debug_result_zoom", m_resultImage(cv::Rect(300, 250, 450, 600)));
		misc::showImage("debug_result", m_resultImage);
	}
}
