/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef IPADETECTION_H
#define IPADETECTION_H

#include <fstream>
#include <opencv2/opencv.hpp>

#include "cameraInterface.hpp"
#include "dbFacade/dbFacade.h"
#include "logging.h"
#include "poseEstimationIPA.h"
#include "udphandler.h"
#include "vs.h"
#include "vs_image.hpp"
#include "yoloDLCPipeline.h"

class IpaDetection
{
  public:
	explicit IpaDetection(const fs::path& cameraConfigPath,
	                      const fs::path& formatConfigPath,
	                      const fs::path& visionSystemConfigPath);
	~IpaDetection();

	/**
	 * @brief run this is a endless loop for processing the images of the pick conveyor
	 *
	 */
	void run();

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

	std::shared_ptr<YoloDLCPipeline> m_yoloDlcPipeline;  // runs detection inference
	YoloDLCResults m_keyPointsAndClasses;                // inference result of yoloDlcPipeline
	std::shared_ptr<db::DbFacade> m_dbFacade;            // handles data base interactions
	std::vector<db::ImprintMap> m_imprintMappingTable;   // maps classId to productID
	uint32_t m_areaNumber = 0;
	bool m_stechstest = false;

	int m_maxAmountOfProductsPerImg = 1;

	std::shared_ptr<PoseEstimationIPA> m_poseEstimationIPA;

	void detectPosesWithNetworks(std::vector<VSPose>& posesInRobotCoordinates,
	                             std::vector<int>& classIds,
	                             eProductsSituationVSComm& prodSituation);
	void estimatePose(const YoloDLCResults& keyPointsAndClasses,
	                  std::vector<VSPose>& posesInRobotCoordinates,
	                  std::vector<int>& classIds);
	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("detectPickableObjectsIPA");
};

#endif  // IPADETECTION_H
