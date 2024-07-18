#ifndef BAUMERCAM_H
#define BAUMERCAM_H
/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * @brief baumer camera class
 *
 */

#include <experimental/filesystem>
#include <opencv2/opencv.hpp>

#include "cameraInterface.hpp"
#include "json.hpp"
#include "logging.h"

namespace fs = std::experimental::filesystem;

struct BaumerCamConfig
{
	float gain = 1.0f;
	float exposureTime = 1000.0f;
	float brightness = 50.0f;
};

class BaumerCam : public VsCameraInterface
{
  public:
	explicit BaumerCam(const fs::path& cameraConfigPath,
	                   const fs::path& networksConfigPath,
	                   const std::string& serialNumber,
	                   bool maxResolution);
	~BaumerCam();

	VsFrame vsGetFrame() override;
	bool init(TriggerMode triggerMode) override;
	int close() override;
	VsCameraIntrinsics vsGetCameraMatrix() override;
	cv::Mat_<double> getExtrinsics() override;
	size_t getImgWidth() override;
	size_t getImgHeight() override;

  protected:
	int m_myFrameID = 0;
	BaumerCamConfig m_baumerConfigs;
	CropConfigs m_cropConfigs;
	class Impl;
	std::shared_ptr<Impl> m_impl;
	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("baumerCam");
};

#endif  // BAUMERCAM_H