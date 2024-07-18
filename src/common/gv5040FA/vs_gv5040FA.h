#ifndef VSGV5040FA_H
#define VSGV5040FA_H

/**
 * @copyright Copyright (c) 2020 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 */
#include <unistd.h>

#include <experimental/filesystem>

#include "cameraInterface.hpp"
#include "logging.h"

namespace fs = std::experimental::filesystem;

struct VsGV5040FAConfig
{
	float gain = 1.0f;
	float exposureTime = 1000.0f;
};

class VsGV5040FA : public VsCameraInterface
{
  public:
	explicit VsGV5040FA(const fs::path& cameraConfigPath,
	                    const fs::path& networksConfigPath,
	                    const std::string& serialNumber,
	                    bool maxResolution);
	~VsGV5040FA();

	VsFrame vsGetFrame() override;
	bool init(TriggerMode triggerMode) override;
	int close() override;
	VsCameraIntrinsics vsGetCameraMatrix() override;
	cv::Mat_<double> getExtrinsics() override;
	size_t getImgWidth() override;
	size_t getImgHeight() override;

  protected:
	class Impl;
	std::unique_ptr<Impl> m_impl;
	int m_myFrameID = 0;
	VsGV5040FAConfig m_idsConfigs;
	CropConfigs m_cropConfigs;
	TriggerMode m_triggerMode;
	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("gv5040FA");
};

#endif  // VSGV5040FA_H
