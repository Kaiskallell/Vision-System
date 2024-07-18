#ifndef DEPTHONLYONDETECTIONS_H
#define DEPTHONLYONDETECTIONS_H

#include <opencv2/opencv.hpp>

#include "logging.h"
#include "projectPaths.h"

struct StereoMaps2
{
	cv::Mat stereoMapL_x;
	cv::Mat stereoMapL_y;
	cv::Mat stereoMapR_x;
	cv::Mat stereoMapR_y;
	cv::Mat rectL;
	cv::Mat Q;

	cv::Mat stereoMapL_inv_x;
	cv::Mat stereoMapL_inv_y;

	cv::cuda::GpuMat gpuMatMapX;
	cv::cuda::GpuMat gpuMatMapY;

	cv::Mat reprojectQ;
	cv::Mat reprojectQFp32;
	cv::cuda::GpuMat stereoMapL_inv_x_gpu;
	cv::cuda::GpuMat stereoMapL_inv_y_gpu;
};

class DepthOnlyOnDetections
{
  public:
	explicit DepthOnlyOnDetections(const fs::path& networksConfigPath);
	~DepthOnlyOnDetections();
	void run(const cv::Mat& imgLeft,
	         const cv::Mat& imgRight,
	         const std::vector<cv::Rect>& detectedBBoxes,
	         cv::Mat& dstDepth);

  private:
	StereoMaps2 m_stereoMaps;

	int m_batchSize = 0;
	int m_windowSize = 0;
	size_t m_inputLayerDimSize = 0;
	size_t m_outputLayerDimSize = 0;
	int32_t m_outputlayerSize0 = 0;
	class Impl;
	std::unique_ptr<Impl> m_impl;
	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("DepthOnlyOnDetections");
};

#endif