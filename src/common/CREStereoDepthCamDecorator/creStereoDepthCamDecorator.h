/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef CRE_STEREO_CAM_DECORATOR_H
#define CRE_STEREO_CAM_DECORATOR_H

#include <experimental/filesystem>

#include "cameraInterface.hpp"
#include "logging.h"
#include "opencv2/opencv.hpp"

namespace fs = std::experimental::filesystem;

class CreStereoCamDecorator : public VsCameraInterface
{
  public:
	explicit CreStereoCamDecorator(const size_t networkConfigIdx,
								   std::vector<std::shared_ptr<VsCameraInterface>>& cameras,
	                               const fs::path& networkConfigPath,
	                               const std::vector<std::string> serialNumbers
								  );
	~CreStereoCamDecorator();
	VsFrame vsGetFrame() override;  // inference happens here

	int close() override;
	bool init(TriggerMode triggerMode) override;
	VsCameraIntrinsics vsGetCameraMatrix() override;
	size_t getImgWidth() override;
	size_t getImgHeight() override;
	cv::Mat_<double> getExtrinsics() override;

  private:
	std::vector<std::shared_ptr<VsCameraInterface>> m_cameras;
	class Impl;  // pimple Ideom to avoid long include chain and compilation time
	std::unique_ptr<Impl> m_impl;
	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("creStereoCamDecorator");
};

#endif
