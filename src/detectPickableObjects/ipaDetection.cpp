/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "ipaDetection.h"

// cameras
#include <unistd.h>

#include <algorithm>  // std::min
#include <experimental/filesystem>

#include "cameraFactory/cameraFactory.hpp"
#include "miscellaneous.h"
#include "mockUdpHandler.hpp"
#include "projectPaths.h"
#include "spdlog/spdlog.h"

namespace visionsystemsignals
{
extern bool shutdownProgam;  // defined in vs_signalHandler.h
}

IpaDetection::IpaDetection(const fs::path& cameraConfigPath,
                           const fs::path& visionSystemConfigPath,
                           const fs::path& networksConfigPath)
{
	// get formatId
	nlohmann::json jsonFile = utils::loadJsonFile(networksConfigPath);
	m_areaNumber = jsonFile.at("areaNumber").get<int>();  // areaNumber means  e.g. Transportstrecke 1 in VMS (camera 1)

	m_camera = setupCamera(visionsystemsignals::shutdownProgam, cameraConfigPath, networksConfigPath, m_kLogger);

	size_t width = m_camera->getImgWidth();
	size_t height = m_camera->getImgHeight();
	m_kLogger->debug("Camera image width: {}", width);
	m_kLogger->debug("Camera image height: {}", height);
	// only the conveyor belt should be visible to the neural network
	m_detectionAreaMask =
	    misc::getDetectionAreaMask(networksConfigPath, m_camera->getSerialNumber(), cv::Size(width, height), m_kLogger);
	// get udp_enabled, check if Vision System should communicate with TwinCat for triggering etc.
	m_udpPickConnectionHandler = udpHander::setupUdpHandler(networksConfigPath, m_kLogger);

	m_poseEstimationIPA = std::make_shared<PoseEstimationIPA>(networksConfigPath);
	fs::path graspPoseEstimationConfigPath = utils::getProjectRootDir() / "src/common/ipaGrip/fc_gqcnn_suction.json";

	m_maxAmountOfProductsPerImg = jsonFile.at("appConfigs").at("generalConfigs").at("maxAmountOfProductsPerImg").get<int>();

	int formatId = std::stoi(networksConfigPath.parent_path().stem().string());
	m_dbFacade = std::make_shared<db::DbFacade>(visionSystemConfigPath);
	db::setupDBObjectForDetection(
	    m_dbFacade, m_imprintMappingTable, visionSystemConfigPath, formatId, m_areaNumber, m_kLogger);
	m_kLogger->debug("end of pickconveyor constructor");
}

IpaDetection::~IpaDetection() { cv::destroyAllWindows(); }

void IpaDetection::detectPosesWithNetworks(std::vector<VSPose>& posesInRobotCoordinates,
                                           std::vector<int>& classIds,
                                           eProductsSituationVSComm& prodSituation)
{
	double dNetwork = (double)cv::getTickCount();

	// Ipa poseEstimation doesnt need any yolo or dlc
	YoloDLCResults keyPointsAndClasses;
	estimatePose(keyPointsAndClasses, posesInRobotCoordinates, classIds);
}

void IpaDetection::estimatePose(const YoloDLCResults& keyPointsAndClasses,
                                std::vector<VSPose>& posesInRobotCoordinates,
                                std::vector<int>& classIds)
{
	double dPose = (double)cv::getTickCount();

	classIds.clear();                 // clear all old results
	posesInRobotCoordinates.clear();  // clear all old results

	// cv::imwrite("src.png", m_frame.colorImage);
	// cv::imwrite("depth.exr", m_frame.depthImage);

	posesInRobotCoordinates = m_poseEstimationIPA->getPoses(
	    m_camera->vsGetCameraMatrix(), m_camera->getExtrinsics(), m_frame.depthImage, m_resultImage);
	constexpr int kMagicOffsetForLAndR = 1;  // objects with classId==0 are not picked by L&R
	for (int i = 0; i < posesInRobotCoordinates.size(); ++i)
	{
		classIds.push_back(0 + kMagicOffsetForLAndR);
		std::cout << "poseRobotCoords = " << posesInRobotCoordinates.at(i).x << " " << posesInRobotCoordinates.at(i).y
		          << " " << posesInRobotCoordinates.at(i).z << " " << posesInRobotCoordinates.at(i).roll << " "
		          << posesInRobotCoordinates.at(i).pitch << " " << posesInRobotCoordinates.at(i).yaw << std::endl;
	}

	// in case someone wants to collect data at the robot for testing purposes
	// misc::saveDetectionsToDisk(posesInRobotCoordinates, m_frame, m_frameNumber, m_kLogger);

	m_kLogger->debug("ESTIMATE_3D_POSE step took {}ms",
	                 (((double)cv::getTickCount() - dPose) / cv::getTickFrequency() * 1000));
}

void IpaDetection::run()
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
		if (!ret || !frameUpdated)
		{
			continue;  // error in the communication or no trigger made
		}

		misc::getFrame(m_frame, m_resultImage, m_camera);  // gets the frame (color+ depth) out of the camera
		if (m_frame.colorImage.empty())
		{
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
