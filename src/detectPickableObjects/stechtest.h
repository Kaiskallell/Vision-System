/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef STECHTEST_H
#define STECHTEST_H

#include <fstream>
#include <opencv2/opencv.hpp>

#include "cameraInterface.hpp"
#include "dbFacade/dbFacade.h"
#include "logging.h"
#include "poseEstimation.h"
#include "udphandler.h"
#include "vs.h"
#include "vs_image.hpp"
#include "vs_c_communicationError.h"

struct StechtestConfigs
{
	bool m_filterByArea = true;
	float m_minArea = 0.0F;
	bool m_filterByCircularity = true;
	float m_minCircularity = 0.0F;
	bool m_filterByConvexity = true;
	float m_minConvexity = 0.0F;
	bool m_filterByInertia = true;
	float m_minInertiaRatio = 0.0F;
};

class Stechtest
{
  public:
	explicit Stechtest(const fs::path& cameraConfigPath,
	                   const fs::path& visionSystemConfigPath,
	                   const fs::path& networksConfigPath);
	~Stechtest();

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
	bool m_udpEnabled = true;

	std::shared_ptr<db::DbFacade> m_dbFacade;           // handles data base interactions
	std::vector<db::ImprintMap> m_imprintMappingTable;  // maps classId to productID
	std::shared_ptr<PoseEstimation> m_poseEstimation;  // Object to make the pose estimation
	uint32_t m_areaNumber = 0;
	size_t m_stechtestPoseIdx = 0;
	int m_maxAmountOfProductsPerImg = 1;
	float m_constZValue = 0.0;
	StechtestConfigs m_stechtestConfigs;

	void detectPosesWithNetworks(std::vector<VSPose>& posesInRobotCoordinates,
	                             std::vector<int>& classIds,
	                             eProductsSituationVSComm& prodSituation);
	void estimatePoses(const std::shared_ptr<yolo::RectsAndClasses> rectsAndClasses,
	                   std::vector<VSPose>& posesInRobotCoordinates,
	                   std::vector<int>& classIds);
	std::shared_ptr<yolo::RectsAndClasses> myBlobDetector(const cv::Mat& image, cv::Mat& debugImg);
	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("stechtest");
};

#endif  // STECHTEST_H
