/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "inferDLC.h"

#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>

#include <fstream>  // std::ifstream

#include "entropyCalibrator.h"
#include "json.hpp"
#include "miscellaneous.h"
#include "trtInference.h"

namespace dlc
{
class InferDLC::Impl
{
  public:
	Impl() = default;
	~Impl() = default;
	std::shared_ptr<trtInf::TrtInference> m_trtInference;
};

DLCConfig InferDLC::readConfig(const fs::path& networksConfigPath, const size_t dlcIdx) const
{
	DLCConfig config = {};

	nlohmann::json jsonFile = utils::loadJsonFile(networksConfigPath);
	try
	{
		config.onnxFilePath = jsonFile.at("appConfigs").at("dlcs").at(dlcIdx).at("generalNetworkConfigs").at("onnxFilePath").get<std::string>();
		config.calibImgsPath = jsonFile.at("appConfigs").at("dlcs").at(dlcIdx).at("generalNetworkConfigs").at("calibImgsPath").get<std::string>();
		config.dataType = jsonFile.at("appConfigs").at("dlcs").at(dlcIdx).at("generalNetworkConfigs").at("dataType").get<std::string>();
		config.confThr = jsonFile.at("appConfigs").at("dlcs").at(dlcIdx).at("detectionConfigs").at("confThr").get<float>();
		config.keyPointsWithHighProbThr = jsonFile.at("appConfigs").at("dlcs").at(dlcIdx).at("detectionConfigs").at("keyPointsWithHighProbThr").get<size_t>();
		config.useTrt = jsonFile.at("appConfigs").at("dlcs").at(dlcIdx).at("generalNetworkConfigs").at("useTrt").get<bool>();
		config.inputLayerName = jsonFile.at("appConfigs").at("dlcs").at(dlcIdx).at("generalNetworkConfigs").at("inputLayerName").get<std::string>();
		config.outputProbsName = jsonFile.at("appConfigs").at("dlcs").at(dlcIdx).at("generalNetworkConfigs").at("outputProbsName").get<std::string>();
		config.outputLocName = jsonFile.at("appConfigs").at("dlcs").at(dlcIdx).at("generalNetworkConfigs").at("outputLocName").get<std::string>();
		config.constAspectRatioCrop = jsonFile.at("appConfigs").at("dlcs").at(dlcIdx).at("generalNetworkConfigs").at("constAspectRatioCrop").get<bool>();
		config.shrinkKeyPointsToCenterEnable =
			jsonFile.at("appConfigs").at("dlcs").at(dlcIdx).at("detectionConfigs").at("shrinkKeyPointsToCenterEnable").get<bool>();
		config.shrinkKeyPointsToCenterVal = jsonFile.at("appConfigs").at("dlcs").at(dlcIdx).at("detectionConfigs").at("shrinkKeyPointsToCenterVal").get<float>();

	}
	catch (const std::exception& e)
	{
		m_kLogger->error("Could not read parameters from file: " + networksConfigPath.string());
		return config;
	}
	
	config.configured = true;

	return config;
};

size_t InferDLC::getInputWidth() { return m_inputWidth; };
size_t InferDLC::getInputHeight() { return m_inputHeight; };

std::string InferDLC::openEngineFile(const fs::path& enginePath) const
{
	std::ifstream file(enginePath.c_str(), std::ios::binary);
	if (!file.good())
	{
		throw std::runtime_error("cannot read engine file! enginePath = " + enginePath.string());
	}

	std::stringstream buffer;
	buffer << file.rdbuf();
	std::string trtModel = buffer.str();

	return std::move(trtModel);  // std::move saves a probably costly string copy
}

InferDLC::InferDLC(const fs::path& networksConfigPath, const size_t dlcIdx)
{
	m_dlcConfig = readConfig(networksConfigPath, dlcIdx);
	m_dlcImpl = std::make_unique<Impl>();

	std::vector<std::string> inputLayerNames;
	inputLayerNames.emplace_back(m_dlcConfig.inputLayerName);
	std::vector<std::string> outputLayerNames;
	outputLayerNames.emplace_back(m_dlcConfig.outputProbsName);
	outputLayerNames.emplace_back(m_dlcConfig.outputLocName);

	m_dlcImpl->m_trtInference = std::make_shared<trtInf::TrtInference>(
	    m_dlcConfig.onnxFilePath, inputLayerNames, outputLayerNames, m_dlcConfig.dataType);

	trtInf::LayerDim32 inputDims = m_dlcImpl->m_trtInference->getLayerDim(inputLayerNames.at(0));
	m_kLogger->debug("inputDims0 = {}x{}x{}x{}", inputDims.d[0], inputDims.d[1], inputDims.d[2], inputDims.d[3]);
	m_inputWidth = inputDims.d[1];
	m_inputHeight = inputDims.d[2];
	m_outputProbSize = m_dlcImpl->m_trtInference->getLayerSize(outputLayerNames.at(0));
	m_outputLocSize = m_dlcImpl->m_trtInference->getLayerSize(outputLayerNames.at(1));

	trtInf::LayerDim32 outputDims0 = m_dlcImpl->m_trtInference->getLayerDim(outputLayerNames.at(0));
	m_kLogger->debug("m_outputProbSize = {}", m_outputProbSize);
	m_kLogger->debug(
	    "outputDims0 = {}x{}x{}x{}", outputDims0.d[0], outputDims0.d[1], outputDims0.d[2], outputDims0.d[3]);
	m_outputProbWidth = outputDims0.d[1];
	m_outputProbHeight = outputDims0.d[2];
	m_outputProbChannels = outputDims0.d[3];

	trtInf::LayerDim32 outputDims1 = m_dlcImpl->m_trtInference->getLayerDim(outputLayerNames.at(1));
	m_kLogger->debug("m_outputLocSize = {}", m_outputLocSize);
	m_kLogger->debug(
	    "outputDims1 = {}x{}x{}x{}", outputDims1.d[0], outputDims1.d[1], outputDims1.d[2], outputDims1.d[3]);
	m_outputLocWidth = outputDims1.d[1];
	m_outputLocHeight = outputDims1.d[2];
	m_outputLocChannels = outputDims1.d[3];

	if (!m_dlcConfig.useTrt)  // no usage of tensorrt
	{
		m_model =
		    cv::dnn::readNetFromONNX(m_dlcConfig.onnxFilePath.string());  // this gives an error: depth_to_space layer
		m_model.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
		m_model.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
	}
};

InferDLC::~InferDLC() { m_kLogger->debug("InferDLC destructor"); }

cv::Mat maskAroundRotRect(const cv::Mat& src, const cv::RotatedRect& rotRect)
{
	cv::Mat mask = cv::Mat::zeros(src.size(), CV_8UC1);
	std::vector<cv::Point2f> pointsRotRect(4);
	rotRect.points(pointsRotRect.data());

	constexpr float kScaleFactor = 1.0F;
	std::vector<cv::Point> pointRotRectInt;
	for (cv::Point2f& point : pointsRotRect)
	{
		point = rotRect.center + (point - rotRect.center) * kScaleFactor;
		pointRotRectInt.emplace_back(cv::Point(point));
	}
	cv::fillConvexPoly(mask, pointRotRectInt, cv::Scalar(255));
	cv::Mat maskedImage;
	src.copyTo(maskedImage, mask);
	return maskedImage;
}

void InferDLC::preProcessing(const cv::Mat& src, const cv::RotatedRect& rotRect, cv::Mat& dst)
{
	cv::Rect rect = rotRect.boundingRect();
	cv::Mat croppedSrc = src(rect);
	cv::RotatedRect rotRectCropped = rotRect;
	rotRectCropped.center.x = rotRect.center.x - rect.x;
	rotRectCropped.center.y = rotRect.center.y - rect.y;
	// mask out (black) everything which is not in rotRect
	cv::Mat maskedImage = maskAroundRotRect(croppedSrc, rotRectCropped);

	if (m_dlcConfig.constAspectRatioCrop)
	{
		constAspectRatioPreprocessing(maskedImage, dst);
	}
	else
	{
		plainResizePreprocessing(maskedImage, dst);
	}
	misc::showImage("dlc_input", dst);
}

void InferDLC::postProcessing(const cv::RotatedRect& rotRect, dlc::PointsAndConfs& pointsAndConfs)
{
	cv::Rect rect = rotRect.boundingRect();
	if (m_dlcConfig.constAspectRatioCrop)
	{
		constAspectRatioPostProcessing(rect, pointsAndConfs);
	}
	else
	{
		plainResizePostProcessing(rect, pointsAndConfs);
	}
}

void InferDLC::plainResizePreprocessing(const cv::Mat& src, cv::Mat& dst)
{
	cv::resize(src, dst, cv::Size(this->getInputWidth(), this->getInputHeight()));
	if (dst.empty())
	{
		throw std::runtime_error("dlc preprocessing dst is empty!");
	}
	cv::cvtColor(dst, dst, cv::COLOR_BGR2GRAY);
	cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR);
}

void InferDLC::plainResizePostProcessing(const cv::Rect& rect, dlc::PointsAndConfs& pointsAndConfs)
{
	size_t dlcWidth = this->getInputWidth();
	size_t dlcHeight = this->getInputHeight();
	// transform keypoints from 128x128 coordinate-system to original img size
	float scaleU = static_cast<float>(rect.width) / static_cast<float>(dlcWidth);
	float scaleV = static_cast<float>(rect.height) / static_cast<float>(dlcHeight);
	for (auto& p : pointsAndConfs.keypoints)
	{
		p.x = (p.x * scaleU) + rect.x;
		p.y = (p.y * scaleV) + rect.y;
	}
}

void InferDLC::constAspectRatioPreprocessing(const cv::Mat& src, cv::Mat& dst)
{
	size_t dlcWidth = this->getInputWidth();
	size_t dlcHeight = this->getInputHeight();

	cv::Mat kernel3 = (cv::Mat_<double>(3, 3) << -1, -1, -1, -1, 9, -1, -1, -1, -1);
	cv::filter2D(src, src, -1, kernel3, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

	int diff = std::abs(src.cols - src.rows);
	m_topCrop = 0;
	m_bottomCrop = 0;
	m_leftCrop = 0;
	m_rightCrop = 0;

	if (src.cols >= src.rows)
	{
		m_topCrop = diff / 2;
		m_bottomCrop = diff / 2;
	}
	else
	{
		m_leftCrop = diff / 2;
		m_rightCrop = diff / 2;
	}

	cv::Scalar color = cv::Scalar(255, 255, 255);
	dst =
	    cv::Mat::zeros(cv::Size(src.cols + m_leftCrop + m_rightCrop, src.rows + m_topCrop + m_bottomCrop), src.type());
	src.copyTo(dst(cv::Rect(m_leftCrop, m_topCrop, src.cols, src.rows)));
	cv::resize(dst, dst, cv::Size(dlcWidth, dlcHeight));
	if (dst.empty())
	{
		throw std::runtime_error("dlcInputImg.empty");
	}
	cv::cvtColor(dst, dst, cv::COLOR_BGR2GRAY);
	cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR);
}

void InferDLC::constAspectRatioPostProcessing(const cv::Rect& rect, dlc::PointsAndConfs& pointsAndConfs)
{
	size_t dlcWidth = this->getInputWidth();
	size_t dlcHeight = this->getInputHeight();
	int longestImgEdge = std::max(rect.width, rect.height);

	// transform keypoints from 128x128 coordinate-system to original img size
	float scaleU = static_cast<float>(longestImgEdge) / static_cast<float>(dlcWidth);
	float scaleV = static_cast<float>(longestImgEdge) / static_cast<float>(dlcHeight);
	for (auto& p : pointsAndConfs.keypoints)
	{
		p.x = (p.x * scaleU) + rect.x - m_leftCrop;
		p.y = (p.y * scaleV) + rect.y - m_topCrop;
	}
}

PointsAndConfs InferDLC::manualLastLayers(std::shared_ptr<float[]> probsOutput,
                                          std::shared_ptr<float[]> locOutput) const
{
	float probsOutputRaw[m_outputProbSize];
	float locOutputRaw[m_outputLocSize];

	memcpy(probsOutputRaw, probsOutput.get(), m_outputProbSize * sizeof(float));
	memcpy(locOutputRaw, locOutput.get(), m_outputLocSize * sizeof(float));

	// find maximum indices in keyPoint-probabilities (column-wise max idxs)
	// same like np.argmax(x, axis=0) in python
	std::vector<int> probsIdxs;
	cv::Mat probs = cv::Mat(cv::Size(m_outputProbChannels, m_outputProbWidth * m_outputProbHeight),
	                        CV_32F,
	                        probsOutputRaw);  // cv::Size(25, 256) = 25 cols and 256 rows
	for (int v = 0; v < probs.cols; ++v)
	{
		float max = 0;
		int maxIdx = 0;
		for (int u = 0; u < probs.rows; ++u)
		{
			if (max < probs.at<float>(u, v))
			{
				max = probs.at<float>(u, v);
				maxIdx = u;
			}
		}
		probsIdxs.push_back(maxIdx);
	}

	// read offsets
	cv::Mat locs = cv::Mat(cv::Size(m_outputProbChannels, m_outputLocHeight * m_outputLocWidth),
	                       CV_32FC2,
	                       locOutputRaw);  // 25,256 but 32FC2 -> 50x256
	std::vector<cv::Point2f> offsets;
	for (int i = 0; i < probsIdxs.size(); ++i)
	{
		float u = locs.at<cv::Vec2f>(probsIdxs.at(i), i)[0];
		float v = locs.at<cv::Vec2f>(probsIdxs.at(i), i)[1];
		offsets.push_back(cv::Point2f(u, v));
	}

	// copy probabilities in extra container
	std::vector<float> probsMaximas;
	for (int i = 0; i < probsIdxs.size(); ++i)
	{
		probsMaximas.push_back(probs.at<float>(probsIdxs[i], i));
	}

	// unravel probIdxs with 16x16 matrix size, see np.unravel function
	std::vector<cv::Point2f> unraveledPoints;
	for (int i = 0; i < probsIdxs.size(); ++i)
	{
		int u = probsIdxs.at(i) % m_outputProbHeight;
		int v = floor(probsIdxs.at(i) / m_outputProbHeight);
		unraveledPoints.push_back(cv::Point2f(u, v));
	}

	// apply magic transformation (last layer of original dlc network)
	// you can get them with netron (use original network not the version
	// modified by us)
	constexpr double kMagicNumber1 = 8.0;
	constexpr double kMagicNumber2 = 4.0;
	constexpr double kMagicNumber3 = 7.2801;

	std::vector<cv::Point2f> keyPoints;
	for (int i = 0; i < probsIdxs.size(); ++i)
	{
		cv::Point2f offsetPoint = offsets[i] * kMagicNumber3;
		keyPoints.push_back(kMagicNumber1 * unraveledPoints[i] + cv::Point2f(kMagicNumber2, kMagicNumber2)
		                    + offsetPoint);
	}

	size_t nHighProbs =
	    std::count_if(probsMaximas.begin(), probsMaximas.end(), [&](float prob) { return prob > m_dlcConfig.confThr; });
	if (nHighProbs < m_dlcConfig.keyPointsWithHighProbThr)
	{
		m_kLogger->debug("Not enough points with heigh probabilities are found");
		return {};  // return empty vector if not enough points with heigh probabilities are found
	}

	PointsAndConfs pointsAndConfs;
	pointsAndConfs.keypoints = keyPoints;
	pointsAndConfs.confs = probsMaximas;
	return std::move(pointsAndConfs);
}

PointsAndConfs InferDLC::run(const cv::Mat& src, const cv::RotatedRect& rotRect)
{
	cv::Mat dlcInputImg;

	preProcessing(src, rotRect, dlcInputImg);
	if (dlcInputImg.empty())
	{
		m_kLogger->error("DLC not able to preprocess image");
		return {};
	}

	if (dlcInputImg.empty())
	{
		throw std::runtime_error("src image is empty!");
	}

	assert(dlcInputImg.channels() == 3);
	assert(dlcInputImg.rows == m_inputHeight);
	assert(dlcInputImg.cols == m_inputWidth);

	cv::Mat floatMat;
	dlcInputImg.convertTo(floatMat, CV_32F);  // values between 0.0F and 255.0F

	std::shared_ptr<float[]> out0 = std::shared_ptr<float[]>(new float[m_outputProbSize]);
	std::shared_ptr<float[]> out1 = std::shared_ptr<float[]>(new float[m_outputLocSize]);
	std::vector<std::shared_ptr<float[]>> outputs = {out0, out1};

	cv::Mat blob;
	int sz[] = {1, 3, static_cast<int>(m_inputHeight), static_cast<int>(m_inputWidth)};
	blob.create(4, sz, CV_32F);
	size_t sizeImage = floatMat.cols * floatMat.rows * floatMat.channels() * sizeof(float);
	memcpy(blob.data, floatMat.data, sizeImage);

	if (m_dlcConfig.useTrt)
	{
		std::vector<trtInf::InputData> blobs;
		trtInf::InputData inputData0;
		inputData0.img = blob;
		inputData0.inputLayerSize = blob.total();
		inputData0.inputIdx = 0;
		blobs.emplace_back(inputData0);
		// Run inference
		m_dlcImpl->m_trtInference->run(blobs, outputs);
	}
	else  // no tensorrt is used
	{
		m_kLogger->debug("No TensorRT is going to be used for inference of dlc");
		bool swapRB = true;
		bool crop = false;
		double scaling = 1.0 / 255.0;
		cv::Scalar mean = cv::Scalar(0, 0, 0);
		cv::Mat blob;
		cv::dnn::blobFromImage(floatMat, blob, scaling, cv::Size(m_inputWidth, m_inputHeight), mean, swapRB, crop);

		std::vector<cv::Mat> outVec;
		std::vector<std::string> outBlobNames = {m_dlcConfig.outputProbsName, m_dlcConfig.outputLocName};
		m_model.setInput(blob);
		m_model.forward(outVec, outBlobNames);
		memcpy(outputs.at(0).get(), outVec[0].data, m_outputProbSize * sizeof(float));
		memcpy(outputs.at(1).get(), outVec[1].data, m_outputLocSize * sizeof(float));
	}

	PointsAndConfs pointsAndConfs = manualLastLayers(outputs.at(0), outputs.at(1));

	// important for longShortEdgePoseEstimaton to avoid large depth values at the edge of a product
	if (m_dlcConfig.shrinkKeyPointsToCenterEnable)
	{
		const float centerX = src.cols * 0.5;
		const float centerY = src.rows * 0.5;
		for (int i = 0; i < pointsAndConfs.keypoints.size(); ++i)
		{
			pointsAndConfs.keypoints.at(i).x =
			    (pointsAndConfs.keypoints.at(i).x - centerX) * m_dlcConfig.shrinkKeyPointsToCenterVal + centerX;
			pointsAndConfs.keypoints.at(i).y =
			    (pointsAndConfs.keypoints.at(i).y - centerY) * m_dlcConfig.shrinkKeyPointsToCenterVal + centerY;
		}
	}
	// transforming the coordinates back to 1024x1024 from 128x128
	postProcessing(rotRect, pointsAndConfs);

	return pointsAndConfs;
}

}  // namespace dlc
