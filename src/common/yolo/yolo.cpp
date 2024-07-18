
/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "yolo.h"

#include <NvOnnxParser.h>

#include <cmath>
#include <fstream>
#include <iostream>

#include "entropyCalibrator.h"
#include "json.hpp"
#include "trtInference.h"

namespace yolo
{
class Yolo::Impl
{
  public:
	Impl() = default;
	~Impl() = default;
	std::shared_ptr<trtInf::TrtInference> m_trtInference;
};

void Yolo::letterBox(const cv::Mat& img,
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
                     double& resizeRatioHeight)
{
	// new/old
	double r =
	    std::min(static_cast<double>(new_shape.width) / img.cols, static_cast<double>(new_shape.height) / img.rows);
	if (!scaleup)
	{
		// only scale down, do not scale up
		r = std::min(r, 1.0);
	}
	// compute padding
	resizeRatioWidth = r;
	resizeRatioHeight = r;
	cv::Size new_unpad = cv::Size(img.cols * r, img.rows * r);
	borderPaddingWidth = new_shape.width - new_unpad.width;
	borderPaddingHeight = new_shape.height - new_unpad.height;

	if (autoSc)
	{
		// minimum rectangle
		borderPaddingWidth = borderPaddingWidth % stride;    // dw padding
		borderPaddingHeight = borderPaddingHeight % stride;  // dh padding
	}
	else if (scaleFill)
	{  // stretch
		borderPaddingWidth = 0;
		borderPaddingHeight = 0;
		new_unpad = new_shape;
		resizeRatioWidth = new_shape.width / img.cols;    // width height ratios
		resizeRatioHeight = new_shape.height / img.rows;  // width height ratios
	}

	borderPaddingWidth /= 2;
	borderPaddingHeight /= 2;

	if (img.size() != new_shape)
	{
		cv::resize(img, dst, new_unpad);
	}
	else
	{
		dst = img.clone();
	}
	int top = borderPaddingHeight;
	int bottom = borderPaddingHeight;
	int left = borderPaddingWidth;
	int right = borderPaddingWidth;
	cv::copyMakeBorder(dst, dst, top, bottom, left, right, cv::BORDER_CONSTANT, color);
}

YoloConfig Yolo::readConfig(const fs::path& networksConfigPath) const
{
	YoloConfig config = {};
	std::ifstream ifs(networksConfigPath.string());
	if (!ifs.good())
	{
		throw std::runtime_error("cannot open yolo config file! networksConfigPath = " + networksConfigPath.string());
	}

	nlohmann::json jsonFile = nlohmann::json::parse(ifs);
	try
	{
		config.onnxPath = jsonFile.at("appConfigs").at("yolo").at("generalNetworkConfigs").at("onnxPath").get<std::string>();
		config.dataType = jsonFile.at("appConfigs").at("yolo").at("generalNetworkConfigs").at("dataType").get<std::string>();
		config.calibImgsPath = jsonFile.at("appConfigs").at("yolo").at("generalNetworkConfigs").at("calibImgsPath").get<std::string>();
		config.nmsThr = jsonFile.at("appConfigs").at("yolo").at("detectionConfigs").at("nmsThr").get<float>();
		config.confThr = jsonFile.at("appConfigs").at("yolo").at("detectionConfigs").at("confThr").get<float>();
		config.useTrt = jsonFile.at("appConfigs").at("yolo").at("generalNetworkConfigs").at("useTrt").get<bool>();
		config.color = jsonFile.at("appConfigs").at("yolo").at("generalNetworkConfigs").at("preprocessing").at("color").get<bool>();
		config.inputLayerName = jsonFile.at("appConfigs").at("yolo").at("generalNetworkConfigs").at("inputLayerName").get<std::string>();
		config.outputLayerName = jsonFile.at("appConfigs").at("yolo").at("generalNetworkConfigs").at("outputLayerName").get<std::string>();
		config.classPredictionOn = jsonFile.at("appConfigs").at("yolo").at("detectionConfigs").at("classPredictionOn").get<bool>();
		config.desiredClasses = jsonFile.at("appConfigs").at("yolo").at("detectionConfigs").at("desiredClasses").get<std::vector<int>>();
		config.whichYoloToUse = jsonFile.at("appConfigs").at("yolo").at("generalNetworkConfigs").at("whichYoloToUse").get<std::string>();
		config.filterBySizeEnable = jsonFile.at("appConfigs").at("yolo").at("detectionConfigs").at("filterBySize").at("enable").get<bool>();
		config.filterBySizeWidthMin = jsonFile.at("appConfigs").at("yolo").at("detectionConfigs").at("filterBySize").at("widthMin").get<float>();
		config.filterBySizeHeightMin = jsonFile.at("appConfigs").at("yolo").at("detectionConfigs").at("filterBySize").at("heightMin").get<float>();
		config.filterBySizeWidthMax = jsonFile.at("appConfigs").at("yolo").at("detectionConfigs").at("filterBySize").at("widthMax").get<float>();
		config.filterBySizeHeightMax = jsonFile.at("appConfigs").at("yolo").at("detectionConfigs").at("filterBySize").at("heightMax").get<float>();
		config.cropOffset = jsonFile.at("appConfigs").at("yolo").at("detectionConfigs").at("cropOffset").get<int>();
		config.sortingDirection = jsonFile.at("appConfigs").at("yolo").at("detectionConfigs").at("sortingDirection").get<std::string>();
		config.sortingRelationOperator = jsonFile.at("appConfigs").at("yolo").at("detectionConfigs").at("sortingRelationOperator").get<std::string>();
	}
	catch (const std::exception& e)
	{
		m_kLogger->error("Could not read parameters from file: " + networksConfigPath.string());
		return config;
	}
	config.configured = true;
	return config;
}

Yolo::Yolo(const fs::path& networksConfigPath)
{
	m_impl = std::make_unique<Impl>();
	m_config = readConfig(networksConfigPath);
	if(!m_config.configured)
	{
		return;
	}
	std::vector<std::string> inputLayerNames;
	inputLayerNames.emplace_back(m_config.inputLayerName);
	std::vector<std::string> outputLayerNames;
	outputLayerNames.emplace_back(m_config.outputLayerName);

	m_impl->m_trtInference =
	    std::make_shared<trtInf::TrtInference>(m_config.onnxPath, inputLayerNames, outputLayerNames, m_config.dataType);

	trtInf::LayerDim32 inputDims = m_impl->m_trtInference->getLayerDim(inputLayerNames.at(0));
	m_kLogger->debug("inputDims0 = {}x{}x{}x{}", inputDims.d[0], inputDims.d[1], inputDims.d[2], inputDims.d[3]);
	m_inputWidth = inputDims.d[2];
	m_inputHeight = inputDims.d[3];
	m_outputlayerSize = m_impl->m_trtInference->getLayerSize(outputLayerNames.at(0));
	trtInf::LayerDim32 outputDims = m_impl->m_trtInference->getLayerDim(outputLayerNames.at(0));
	m_kLogger->debug("m_outputlayerSize = {}", m_outputlayerSize);
	m_kLogger->debug("outputDims0 = {}x{}x{}x{}", outputDims.d[0], outputDims.d[1], outputDims.d[2], outputDims.d[3]);
	m_outputWidth = outputDims.d[2];
	m_outputHeight = outputDims.d[1];

	if (!m_config.useTrt)  // no usage of tensorrt
	{
		m_model = cv::dnn::readNetFromONNX(m_config.onnxPath.string());
		m_model.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
		m_model.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
		// m_model.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
		// m_model.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
	}
}

Yolo::~Yolo() { m_kLogger->debug("yolo destructor"); }

bool Yolo::sortAlongDirection(std::pair<cv::RotatedRect, ClassId>& a,
                              std::pair<cv::RotatedRect, ClassId>& b,
                              const std::string& sortingDirection,
                              const std::string& sortingRelationOperator)
{
	bool ret = false;

	if (sortingDirection == "v")
	{
		int yCenterOfA = a.first.center.y;
		int yCenterOfB = b.first.center.y;
		if (sortingRelationOperator == ">")
		{
			ret = yCenterOfA > yCenterOfB;
		}
		else if (sortingRelationOperator == "<")
		{
			ret = yCenterOfA < yCenterOfB;
		}
		else
		{
			m_kLogger->error("sortingRelationOperator is not valid");
		}
	}
	else if (sortingDirection == "u")
	{
		int xCenterOfA = a.first.center.x;
		int xCenterOfB = b.first.center.x;
		if (sortingRelationOperator == ">")
		{
			ret = xCenterOfA > xCenterOfB;
		}
		else if (sortingRelationOperator == "<")
		{
			ret = xCenterOfA < xCenterOfB;
		}
		else
		{
			m_kLogger->error("sortingRelationOperator is not valid");
		}
	}
	return ret;
}

void Yolo::yoloV5RawDataToObjects(const cv::Mat& netOutput,
                                  std::vector<cv::RotatedRect>& boxes,
                                  std::vector<float>& scores,
                                  std::vector<int>& classes)
{
	for (int r = 0; r < netOutput.size[0]; r++)
	{
		float cx = netOutput.at<float>(r, 0);
		float cy = netOutput.at<float>(r, 1);
		float w = netOutput.at<float>(r, 2);
		float h = netOutput.at<float>(r, 3);
		float sc = netOutput.at<float>(r, 4);
		if (sc >= m_config.confThr)
		{
			cv::Mat confs = netOutput.row(r).colRange(5, netOutput.size[1]);  // (5,7) in case of two classes
			confs *= sc;
			// finding the class with maximum confidence
			int minIdx = 0;
			int maxIdx = 0;
			for (int k = 0; k < confs.size().width; k++)
			{
				if (confs.at<float>(0, k) > confs.at<float>(0, maxIdx))
				{
					maxIdx = k;
				}
			}
			classes.emplace_back(maxIdx);
			scores.emplace_back(sc);
			boxes.emplace_back(cv::RotatedRect(cv::Point2f(cx, cy), cv::Size2f(w, h), 0.0));
		}
	}
}

void Yolo::yoloV8RawDataToObjects(const cv::Mat& netOutput,
                                  std::vector<cv::RotatedRect>& boxes,
                                  std::vector<float>& confidences,
                                  std::vector<int>& classes)
{
	for (int c = 0; c < netOutput.size[1]; c++)
	{
		float cx = netOutput.at<float>(0, c);
		float cy = netOutput.at<float>(1, c);
		float w = netOutput.at<float>(2, c);
		float h = netOutput.at<float>(3, c);

		cv::Mat confs = netOutput.col(c).rowRange(4, netOutput.size[0]);
		cv::Point class_id;
		double maxClassScore;
		cv::minMaxLoc(confs, 0, &maxClassScore, 0, &class_id);
		if (maxClassScore >= m_config.confThr)
		{
			classes.emplace_back(class_id.y);
			confidences.emplace_back(maxClassScore);
			boxes.emplace_back(cv::RotatedRect(cv::Point2f(cx, cy), cv::Size2f(w, h), 0.0));
		}
	}
}

void Yolo::yoloRotBBOxRawDataToObjects(const cv::Mat& netOutput,
                                       std::vector<cv::RotatedRect>& boxes,
                                       std::vector<float>& confidences,
                                       std::vector<int>& classes)
{
	for (int c = 0; c < netOutput.size[1]; c++)
	{
		float cx = netOutput.at<float>(0, c);
		float cy = netOutput.at<float>(1, c);
		float w = netOutput.at<float>(2, c);
		float h = netOutput.at<float>(3, c);
		float angleInDegree = netOutput.at<float>(netOutput.size[0]-1, c) * 180.0 / M_PI;

		cv::Mat confs = netOutput.col(c).rowRange(4, netOutput.size[0]-1);
		cv::Point class_id;
		double maxClassScore;
		cv::minMaxLoc(confs, 0, &maxClassScore, 0, &class_id);  // find max class score and its class id

		if (maxClassScore >= m_config.confThr)
		{
			classes.emplace_back(class_id.y);
			confidences.emplace_back(maxClassScore);
			boxes.emplace_back(cv::RotatedRect(cv::Point2f(cx, cy), cv::Size2f(w, h), angleInDegree));
		}
	}
}

bool Yolo::cropFilter(cv::Mat& src,
					  cv::RotatedRect& box)
{	
	box.center.x = box.center.x / m_resizeRatioWidth
		                      - m_borderPaddingWidth;  // transform back to original img size 1440x1440 -> 1440x1080
	box.center.y = box.center.y / m_resizeRatioHeight - m_borderPaddingHeight;
	box.size.width /= m_resizeRatioWidth;
	box.size.height /= m_resizeRatioHeight;
	//crop
	box.size.height = box.size.height + m_config.cropOffset;
	box.size.width = box.size.width + m_config.cropOffset; 
	cv::Rect rect = box.boundingRect();
	if (rect.x < 0 || rect.y < 0 || rect.x + rect.width > src.cols || rect.y + rect.height > src.rows)
	{
		m_kLogger->error("cropping not possible. Skipping this rectangle");
		m_kLogger->error("Did you load the correct format/networks?");
		return false;  // rect is at least partly outside of image
	}
	return true;
}

std::shared_ptr<RectsAndClasses> Yolo::run(cv::Mat& src, cv::Mat& debugImg)
{
	// auto start = std::chrono::system_clock::now();
	if (src.empty())
	{
		throw std::runtime_error("src image is empty!");
	}

	if (m_config.color && src.channels() == 1)
	{
		m_kLogger->warn(
		    "check your config files for color options in camera and yolo config! src img is single channel");
		cv::cvtColor(src, src, cv::COLOR_GRAY2BGR);
	}

	if (!m_config.color)  // convert from 3 channel color to 3 channel gray
	{
		// in case we need a gray scale input image
		if (src.channels() != 1)
		{
			cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
		}
		cv::cvtColor(src, src, cv::COLOR_GRAY2BGR);
	}

	cv::Mat networkInputImg;
	cv::Size newShape = cv::Size(m_inputWidth, m_inputHeight);
	cv::Scalar color = cv::Scalar(114, 114, 114);
	
	letterBox(src,
	          newShape,
	          color,
	          false,
	          false,
	          true,
	          64,
	          networkInputImg,
	          m_borderPaddingWidth,
	          m_borderPaddingHeight,
	          m_resizeRatioWidth,
	          m_resizeRatioHeight);  // it draws a grey (color) border in the area where it is resized

	bool swapRB = true;
	bool crop = false;
	double scaling = 1.0 / 255.0;
	cv::Scalar mean = cv::Scalar(0, 0, 0);
	cv::Mat blob;
	cv::dnn::blobFromImage(networkInputImg, blob, scaling, cv::Size(m_inputWidth, m_inputHeight), mean, swapRB, crop);

	cv::Mat out;
	if (m_config.useTrt)
	{
		std::shared_ptr<float[]> out0 = std::shared_ptr<float[]>(new float[m_outputlayerSize]);
		std::vector<std::shared_ptr<float[]>> outputs = {out0};
		std::vector<trtInf::InputData> blobs;
		trtInf::InputData inputData0;
		inputData0.img = blob;
		inputData0.inputLayerSize = blob.total();
		inputData0.inputIdx = 0;
		blobs.emplace_back(inputData0);
		// Run inference
		m_impl->m_trtInference->run(blobs, outputs);  // inference
		out = cv::Mat(m_outputHeight, m_outputWidth, CV_32F, outputs.at(0).get());
	}
	else  // no tensorrt is used
	{
		m_kLogger->debug("No TensorRT is going to be used for inference of Yolo");
		m_model.setInput(blob);
		out = m_model.forward(m_kOutputBlobName);                           // 1x127575x7
		out = cv::Mat(out.size[1], out.size[2], CV_32F, out.ptr<float>());  // 127575x7, no batchsize anymore
	}

	// post processing
	std::vector<cv::RotatedRect> boxes;
	std::vector<float> scores;
	std::vector<int> classes;

	if (m_config.whichYoloToUse == "yoloV5")
	{
		yoloV5RawDataToObjects(out, boxes, scores, classes);
	}
	else if (m_config.whichYoloToUse == "yoloV8")
	{
		yoloV8RawDataToObjects(out, boxes, scores, classes);
	}
	else if (m_config.whichYoloToUse == "yoloRotBBox")
	{
		yoloRotBBOxRawDataToObjects(out, boxes, scores, classes);
	}
	else
	{
		m_kLogger->error("yoloType in config file not valid: {}", m_config.whichYoloToUse);
	}

	std::vector<int> indices;
	cv::dnn::NMSBoxes(boxes, scores, m_config.confThr, m_config.nmsThr, indices);

	std::shared_ptr<RectsAndClasses> detections = std::make_shared<RectsAndClasses>();
	for (auto& ind : indices)
	{
		if(!cropFilter(src, boxes[ind]))//crop is outside of the image
		{
			continue;
		}
		std::string text = "class=" + std::to_string(classes[ind]) + " conf=" + std::to_string(scores[ind]);
		cv::Point2f vertices[4];  // for only for drawing
		boxes[ind].points(vertices);
		for (int i = 0; i < 4; ++i)
		{
			line(debugImg, vertices[i], vertices[(i + 1) % 4], cv::Scalar(128, 223, 130), 10);
		}

		cv::putText(debugImg,
		            text,
		            cv::Point(boxes[ind].center.x, boxes[ind].center.y),
		            cv::FONT_HERSHEY_DUPLEX,
		            1,
		            cv::Scalar(0, 255, 0),
		            2,
		            false);
		if(m_config.filterBySizeEnable)
		{
			// sometimes are yolo detections instable, caused by not proper belt
			if(boxes[ind].size.width < m_config.filterBySizeWidthMin || 
			   boxes[ind].size.height < m_config.filterBySizeHeightMin ||
			   boxes[ind].size.width > m_config.filterBySizeWidthMax ||
			   boxes[ind].size.height > m_config.filterBySizeHeightMax)
			{
				continue;
			}
		}		
		if (m_config.classPredictionOn)
		{
			detections->emplace_back(std::make_pair(boxes[ind], classes[ind]));
		}
		else
		{
			detections->emplace_back(
			    std::make_pair(boxes[ind], 0));  // no turning station, because everything is considered as front side
		}
	}

	std::shared_ptr<RectsAndClasses> filteredDetections = std::make_shared<RectsAndClasses>();

	for (int i = 0; i < detections->size(); ++i)
	{
		for (int j = 0; j < m_config.desiredClasses.size(); ++j)
		{
			if (m_config.desiredClasses.at(j) == detections->at(i).second)
			{
				// found a desired class in detection
				filteredDetections->emplace_back(detections->at(i));
				break;  // no need to go through whole loop
			}
		}
	}

	using namespace std::placeholders;
	std::sort(filteredDetections->begin(),
	          filteredDetections->end(),
	          std::bind(sortAlongDirection, _1, _2, m_config.sortingDirection, m_config.sortingRelationOperator));
	
	return filteredDetections;
}

}  // namespace yolo
