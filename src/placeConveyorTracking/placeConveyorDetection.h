/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved

 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 *
 */

#ifndef PLACECONVDETECTION_H
#define PLACECONVDETECTION_H

#include <opencv2/opencv.hpp>

#include "cameraInterface.hpp"
#include "dbFacade/dbFacade.h"
#include "logging.h"
#include "poseEstimation.h"
#include "projectPaths.h"
#include "trackingInterface.h"
#include "udphandler.h"
#include "vs.h"
#include "vs_image.hpp"
#include "yoloDLCPipeline.h"
#include "vs_c_communicationError.h"

class PlaceConvDetection
{
  public:
	PlaceConvDetection(const fs::path& cameraConfigPath,
	                   const fs::path& visionSystemConfigPath,
	                   const fs::path& networksConfigPath);
	~PlaceConvDetection();
	bool m_moduleConfigured = false;
	void run(std::shared_ptr<InterfaceDetTrk> interfaceDetTrk);

  private:
	void detectPosesWithNetworks(std::vector<VSPose>& posesInRobotCoordinates, std::vector<int>& classIds);
	void estimatePose(YoloDLCResults& keyPointsAndClasses,
	                  std::vector<VSPose>& posesInRobotCoordinates,
	                  std::vector<int>& classIds);

	std::shared_ptr<VsCameraInterface> m_camera;

	cv::Mat m_resultImage;  // for debug output
	VsFrame m_frame;        // hold images from cameraInterface

	cv::Mat m_detectionAreaMask;
	std::shared_ptr<UdpHandler> m_udpPickConnectionHandler;  // makes communication with TwinCat3
	uint32_t m_rotEncoderValue =
	    0;  // rotation encoder value from TwinCat3 received over udp and send to VsCommunication to VMS
	uint64_t m_frameNumber = 0;
	bool m_frameNumberOverflow = false;
	std::shared_ptr<PoseEstimation> m_poseEstimation;  // Object to make the pose estimation
	bool m_udpEnabled = true;

	std::shared_ptr<YoloDLCPipeline> m_yoloDlcPipeline;  // runs detection inference
	YoloDLCResults m_keyPointsAndClasses;                // inference result of yoloDlcPipeline
	std::shared_ptr<db::DbFacade> m_dbFacade;            // handles data base interactions
	std::vector<db::ImprintMap> m_imprintMappingTable;   // maps classId to productID
	uint32_t m_areaNumber = 0;
	bool m_stechstest = false;

	int m_maxAmountOfProductsPerImg = 1;

	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("placeConveyorDetection");
};

#endif
