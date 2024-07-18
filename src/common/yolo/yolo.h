
/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef YOLO_H
#define YOLO_H

#include <experimental/filesystem>
#include <opencv2/opencv.hpp>

#include "logging.h"

namespace fs = std::experimental::filesystem;

namespace yolo
{
struct YoloConfig
{
	fs::path onnxPath = "";
	fs::path calibImgsPath = "";
	float confThr = 0.0f;
	float nmsThr = 0.0f;
	std::string dataType = "";
	std::string sortingDirection = "";
	std::string sortingRelationOperator = "";
	std::string inputLayerName = "";
	std::string outputLayerName = "";
	std::string whichYoloToUse = "";
	bool useTrt = true;
	bool color = false;
	bool classPredictionOn = false;
	int cropOffset = 0;
	std::vector<int> desiredClasses;
	bool filterBySizeEnable = false;
	float filterBySizeWidthMin = 0.0;
	float filterBySizeWidthMax = 0.0;
	float filterBySizeHeightMin = 0.0;
	float filterBySizeHeightMax = 0.0;
	bool configured = false;
};

enum class Modus
{
	Pick = 0,
	Place = 1
};

typedef int ClassId;
typedef std::vector<std::pair<cv::RotatedRect, ClassId>> RectsAndClasses;

class Yolo
{
  public:
	Yolo(const fs::path& networksConfigPath);
	~Yolo();
	YoloConfig m_config;
	std::shared_ptr<RectsAndClasses> run(cv::Mat& src, cv::Mat& debugImg);

  private:
	YoloConfig readConfig(const fs::path& networksConfigPath) const;

	// this function is a cpp implementation from yolov5/utils/augmentations.py
	void letterBox(const cv::Mat& img,
	               const cv::Size& new_shape,
	               const cv::Scalar& color,
	               const bool autoSc,
	               const bool scaleFill,
	               const bool scaleup,
	               const int stride,
	               cv::Mat& dst,
	               int& borderPaddingWidth,
	               int& borderPaddingHeight,
	               double& resizeRatioWidth,
	               double& resizeRatioHeight);
	static bool sortAlongDirection(std::pair<cv::RotatedRect, ClassId>& a,
	                               std::pair<cv::RotatedRect, ClassId>& b,
	                               const std::string& sortingDirection,
	                               const std::string& sortingRelationOperator);
	void yoloV8RawDataToObjects(const cv::Mat& netOutput,
	                            std::vector<cv::RotatedRect>& boxes,
	                            std::vector<float>& confidences,
	                            std::vector<int>& classes);
	void yoloV5RawDataToObjects(const cv::Mat& netOutput,
	                            std::vector<cv::RotatedRect>& boxes,
	                            std::vector<float>& scores,
	                            std::vector<int>& classes);
	void yoloRotBBOxRawDataToObjects(const cv::Mat& netOutput,
	                                 std::vector<cv::RotatedRect>& boxes,
	                                 std::vector<float>& confidences,
	                                 std::vector<int>& classes);
	bool cropFilter(cv::Mat& src,
					cv::RotatedRect& box);

	class Impl;
	std::unique_ptr<Impl> m_impl;

	const size_t m_kBatchsize = 1;
	size_t m_inputHeight = 0;
	size_t m_inputWidth = 0;
	size_t m_outputHeight = 0;
	size_t m_outputWidth = 0;
	int32_t m_outputlayerSize = 0;
	const char* m_kInputBlobName = "images";
	const char* m_kOutputBlobName = "output";
	int m_borderPaddingWidth = -1;
	int m_borderPaddingHeight = -1;
	double m_resizeRatioWidth = 0.0;
	double m_resizeRatioHeight = 0.0;

	cv::dnn::Net m_model;  // only for the case when no tensorrt is used

	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("yolo");
};
}  // namespace yolo

#endif
