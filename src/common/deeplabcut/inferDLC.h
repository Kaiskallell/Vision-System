
/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef INFER_DLC_H
#define INFER_DLC_H

#include <experimental/filesystem>
#include <opencv2/opencv.hpp>

#include "logging.h"

namespace fs = std::experimental::filesystem;

namespace dlc
{
struct DLCConfig
{
	fs::path onnxFilePath = "";
	fs::path calibImgsPath = "";
	std::string dataType = "";  // int8, fp16,fp32
	float confThr = 0.0f;
	size_t keyPointsWithHighProbThr = 0;
	bool useTrt = true;
	std::string inputLayerName = "";
	std::string outputLayerName = "";
	std::string outputProbsName = "";
	std::string outputLocName = "";
	bool constAspectRatioCrop = false;
	bool shrinkKeyPointsToCenterEnable = false;
	float shrinkKeyPointsToCenterVal = 1.0;
	bool configured = false;
};

struct PointsAndConfs
{
	std::vector<cv::Point2f> keypoints;  // 25 keypoints
	std::vector<float> confs;            // 25 confidences
};

class InferDLC
{
  public:
	InferDLC(const fs::path& networksConfigPath, const size_t dlcIdx);
	~InferDLC();
	DLCConfig m_dlcConfig;
	PointsAndConfs run(const cv::Mat& src, const cv::RotatedRect& rotRect);  // inference function
	size_t getInputWidth();
	size_t getInputHeight();
	void preProcessing(const cv::Mat& src, const cv::RotatedRect& rotRect, cv::Mat& dst);
	void postProcessing(const cv::RotatedRect& rotRect, dlc::PointsAndConfs& pointsAndConfs);

  private:
	DLCConfig readConfig(const fs::path& networksConfigPath, const size_t dlcIdx) const;
	std::string openEngineFile(const fs::path& enginePath) const;

	/**
	 * @brief This function was adapted from the previous python script. It replaces the last layers of DLC because
	 * these used to make problems with tensorrt optimization
	 */
	PointsAndConfs manualLastLayers(std::shared_ptr<float[]> probsOutput, std::shared_ptr<float[]> locOutput) const;

	void plainResizePreprocessing(const cv::Mat& src, cv::Mat& dst);
	void plainResizePostProcessing(const cv::Rect& rect, dlc::PointsAndConfs& pointsAndConfs);
	void constAspectRatioPreprocessing(const cv::Mat& src, cv::Mat& dst);
	void constAspectRatioPostProcessing(const cv::Rect& rect, dlc::PointsAndConfs& pointsAndConfs);

	class Impl;
	std::unique_ptr<Impl> m_dlcImpl;


	int m_inputIndex = 0;
	int m_outputIndexProb = 1;
	int m_outputIndexLoc = 2;
	float* m_img_host = nullptr;
	float* m_img_device = nullptr;

	size_t m_inputHeight = 128;
	size_t m_inputWidth = 128;

	int32_t m_outputProbSize = 0;
	int32_t m_outputLocSize = 0;

	// output layers stuff
	size_t m_outputProbHeight = 16;
	size_t m_outputProbWidth = 16;
	size_t m_outputProbChannels = 25;
	size_t m_outputLocHeight = 16;
	size_t m_outputLocWidth = 16;
	size_t m_outputLocChannels = 50;

	float* m_buffers[3] = {};  // for input and 2*output of network
	cv::dnn::Net m_model;      // only for the case when no tensorrt is used

	int m_topCrop = 0;
	int m_bottomCrop = 0;
	int m_leftCrop = 0;
	int m_rightCrop = 0;

	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("dlc");
};
}  // namespace dlc

#endif
