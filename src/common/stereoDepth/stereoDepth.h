/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef STEREO_DEPTH_H
#define STEREO_DEPTH_H

#include <experimental/filesystem>

#include "logging.h"
#include "opencv2/opencv.hpp"

namespace fs = std::experimental::filesystem;

struct StereoMaps
{
	cv::Mat stereoMapL_x;
	cv::Mat stereoMapL_y;
	cv::Mat stereoMapR_x;
	cv::Mat stereoMapR_y;
	cv::Mat rectL;
	cv::Mat Q;

	cv::Mat stereoMapL_inv_x;
	cv::Mat stereoMapL_inv_y;
	
};

class StereoDepth
{
  public:
	StereoDepth(const fs::path& networksConfigPath, const size_t networkConfigIdx);
	~StereoDepth();
	cv::Mat run(const cv::Mat& srcLeft, const cv::Mat& srcRight);

  protected:
	void postProcessing(const cv::Mat& disparityImg, cv::Mat& dst, const cv::Mat& Q, const cv::Mat& rectL);
	void preProcessing(const cv::Mat& src,
	                   cv::Mat& dst,  // const cv::Rect &cropRect,
	                   const cv::Mat& mapX,
	                   const cv::Mat& mapY,
	                   const cv::Size& resize);
	StereoMaps loadStereoCalibParams(const fs::path& paramPath);

	class Impl;
	std::unique_ptr<Impl> m_impl;
	StereoMaps m_stereoMaps;
	size_t m_inputWidth = 0;
	size_t m_inputHeight = 0;
	size_t m_outputWidth = 0;
	int32_t m_outputHeight = 0;
	int32_t m_outputlayerSize0 = 0;

	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("stereoDepth");
};

#endif
