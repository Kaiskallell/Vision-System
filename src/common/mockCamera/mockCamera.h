/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include <experimental/filesystem>

#include "cameraInterface.hpp"
#include "logging.h"
#include "vs_image.hpp"

namespace fs = std::experimental::filesystem;

struct Detection
{
	float x;
	float y;
	float z;
	float roll;
	float pitch;
	float yaw;
};

struct LoadedImage
{
	std::string color_image_path;
};

class MockCamera : public VsCameraInterface
{
  public:
	MockCamera(const fs::path& cameraConfigPath, const fs::path& networksConfigPath, const std::string& serialNumber);
	~MockCamera();

	VsFrame vsGetFrame() override;
	bool init(TriggerMode triggerMode) override;
	int close() override;
	VsCameraIntrinsics vsGetCameraMatrix() override;
	cv::Mat_<double> getExtrinsics() override;
	size_t getImgWidth() override;
	size_t getImgHeight() override;

  private:
	std::vector<LoadedImage> m_imagesPaths;
	std::vector<LoadedImage>::iterator current_image;
	std::vector<Detection> detections;
	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("mockCamera");
	CropConfigs m_cropConfigs;
	std::vector<std::string> getFilenames(std::experimental::filesystem::path path);
};
