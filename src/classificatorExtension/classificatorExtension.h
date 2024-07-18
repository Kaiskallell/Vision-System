/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 *
 */

#ifndef CLASSIFICATIONEXTENSION_H
#define CLASSIFICATIONEXTENSION_H

#pragma once

#include "cameraInterface.hpp"
#include "dbFacade/dbFacade.h"
#include "logging.h"
#include "projectPaths.h"
#include "udphandler.h"
#include "vs.h"
#include "vs_image.hpp"
#include "yolo/yolo.h"
#include "vs_c_communicationError.h"

class ClassificationExtension
{
  public:
	ClassificationExtension(const fs::path& cameraConfigPath,
	                        const fs::path& visionSystemConfigPath,
	                        const fs::path& networksConfigPath);
	~ClassificationExtension();
	bool run();
	
  private:
	void detectPosesWithNetworks(int& classId, eProductsSituationVSComm& prodSituation);
	void detectPosesWithNetworks(int& classId);
	void runTC3(); //No need in the future
	void runVMS();
	void runDebug(); //This is needed because when using VMS, it is also SoftwareTrigger but not continuous and does not suit for debugging purpose

	std::shared_ptr<VsCameraInterface> m_camera;

	cv::Mat m_resultImage;  // for debug output
	VsFrame m_frame;        // hold images from cameraInterface

	cv::Mat m_detectionAreaMask;
	std::shared_ptr<UdpHandler> m_udpClassificatprExtensionConnectionHandler ;  // makes communication with TwinCat3

	uint32_t m_rotEncoderValue =
	    0;  // rotation encoder value from TwinCat3 received over udp and send to VsCommunication to VMS
	uint64_t m_frameNumber = 0;

	bool m_frameNumberOverflow = false;	
	bool m_extReqOld = false;

	std::shared_ptr<db::DbFacade> m_dbFacade;           // handles data base interactions
	std::vector<db::ImprintMap> m_imprintMappingTable;  // maps classId to productID
	std::unique_ptr<yolo::Yolo> m_yolo;
	uint32_t m_areaNumber = 0;
	bool m_vms_enabled = false;
	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("classificationExtension");
};

#endif
