/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef DETECT_FROM_TRAY_H
#define DETECT_FROM_TRAY_H

#include <fstream>
#include <opencv2/opencv.hpp>

#include "cameraInterface.hpp"
#include "dbFacade/dbFacade.h"
#include "logging.h"
#include "poseEstimation.h"
#include "udphandler.h"
#include "vs.h"
#include "vs_image.hpp"
#include "yolo/yolo.h"
#include "vs_c_communicationError.h"

class DetectFromTray
{
  public:
	explicit DetectFromTray(const fs::path& cameraConfigPath,
	                        const fs::path& visionSystemConfigPath,
	                        const fs::path& networksConfigPath);
	~DetectFromTray();

	/**
	 * @brief run this is a endless loop for processing the images of the pick conveyor
	 *
	 */
	bool m_moduleConfigured = false;
	virtual void run();

  protected:
	std::shared_ptr<VsCameraInterface> m_camera;

	cv::Mat m_resultImage;  // for debug output
	VsFrame m_frame;        // hold images from cameraInterface

	cv::Mat m_detectionAreaMask;
	std::shared_ptr<UdpHandler> m_udpPickConnectionHandler;  // makes communication with TwinCat3
	uint32_t m_rotEncoderValue =
	    0;  // rotation encoder value from TwinCat3 received over udp and send to VsCommunication to VMS
	uint64_t m_frameNumber = 0;
	bool m_frameNumberOverflow = false;
	bool m_constZEnable = false;
	float m_constZValue = 0.0f;
	std::vector<cv::Point> m_firstTrayPolygon;

	bool m_udpEnabled = true;

	std::shared_ptr<db::DbFacade> m_dbFacade;           // handles data base interactions
	std::vector<db::ImprintMap> m_imprintMappingTable;  // maps classId to productID
	std::unique_ptr<yolo::Yolo> m_yolo;
	std::shared_ptr<PoseEstimation> m_poseEstimation;  // Object to make the pose estimation
	uint32_t m_areaNumber = 0;

	int m_maxAmountOfProductsPerImg = 1;

	void detectPosesWithNetworks(std::vector<VSPose>& posesInRobotCoordinates,
	                             std::vector<int>& classIds,
	                             eProductsSituationVSComm& prodSituation);
	void estimatePoses(const std::shared_ptr<yolo::RectsAndClasses> rectsAndClasses,
	                   std::vector<VSPose>& posesInRobotCoordinates,
	                   std::vector<int>& classIds);
	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("detectFromTray");
};

#endif  // DETECT_FROM_TRAY_H
