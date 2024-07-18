/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "stereoDepth.h"

#include <experimental/filesystem>

#include "projectPaths.h"
#include "trtInference.h"

class StereoDepth::Impl
{
  public:
	Impl() = default;
	~Impl() = default;
	std::shared_ptr<trtInf::TrtInference> m_trtInference;
};

StereoMaps StereoDepth::loadStereoCalibParams(const fs::path& paramPath)
{
	// load xml file
	cv::FileStorage fileStorage(paramPath.string(), cv::FileStorage::READ);

	StereoMaps stereoMaps;
	// load mapX & mapY, for rectification of stereo camera pair
	stereoMaps.stereoMapL_x = fileStorage["stereoMapL_x"].mat();
	stereoMaps.stereoMapL_y = fileStorage["stereoMapL_y"].mat();
	stereoMaps.stereoMapR_x = fileStorage["stereoMapR_x"].mat();
	stereoMaps.stereoMapR_y = fileStorage["stereoMapR_y"].mat();

	stereoMaps.stereoMapL_inv_x = fileStorage["stereoMapL_inv_x"].mat();
	stereoMaps.stereoMapL_inv_y = fileStorage["stereoMapL_inv_y"].mat();

	// for unrectify imgs (last step before passing to e.g. yolo)
	stereoMaps.rectL = fileStorage["rectL"].mat();

	// load Q, transform disparity to 3d coordinates [mm]
	stereoMaps.Q = fileStorage["Q"].mat();

	return stereoMaps;
}

StereoDepth::StereoDepth(const fs::path& networksConfigPath, const size_t networkConfigIdx)
{
	m_impl = std::make_unique<Impl>();

	// loading from config file in section "StereoDepth"
	std::string networkType = "stereoDepth";
	nlohmann::json jsonFile = utils::loadJsonFile(networksConfigPath);
	// assign default value
	fs::path mappingPath = jsonFile.at("cameraConfigs").at("camera")[networkConfigIdx].at(networkType).at("mappingConfigPath").get<std::string>();
	std::string inputLayerName0 = "";
	std::string inputLayerName1 = "";
	std::string outputLayerName1 = "";
	fs::path onnxPath = "";
	std::string inferDataType = "";
	fs::path calibImgsPath0 = "";
	fs::path calibImgsPath1 = "";
	try
	{
		mappingPath = jsonFile.at("cameraConfigs").at("camera")[networkConfigIdx].at(networkType).at("mappingConfigPath").get<std::string>();
		inputLayerName0 =
		jsonFile.at("cameraConfigs").at("camera")[networkConfigIdx].at(networkType).at("inputLayerName0").get<std::string>();  // needs to be "left"
		inputLayerName1 =
			jsonFile.at("cameraConfigs").at("camera")[networkConfigIdx].at(networkType).at("inputLayerName1").get<std::string>();  // needs to be "right"
		outputLayerName1 =
			jsonFile.at("cameraConfigs").at("camera")[networkConfigIdx].at(networkType).at("outputLayerName1").get<std::string>();  // needs to be "output"
		onnxPath = jsonFile.at("cameraConfigs").at("camera")[networkConfigIdx].at(networkType).at("onnxPath").get<std::string>();
		inferDataType = jsonFile.at("cameraConfigs").at("camera")[networkConfigIdx].at(networkType).at("dataType").get<std::string>();
		calibImgsPath0 = jsonFile.at("cameraConfigs").at("camera")[networkConfigIdx].at(networkType).at("calibImgsPath0").get<std::string>();
		calibImgsPath1 = jsonFile.at("cameraConfigs").at("camera")[networkConfigIdx].at(networkType).at("calibImgsPath1").get<std::string>();
	}
	catch (const std::exception& e)
	{
		m_kLogger->error("Could not read parameters from file: " + networksConfigPath.string());
	}
	
	const std::vector<fs::path>& calibImgsFolders = {calibImgsPath0, calibImgsPath1};

	m_stereoMaps = loadStereoCalibParams(mappingPath);

	std::vector<std::string> inputLayerNames;
	inputLayerNames.emplace_back(inputLayerName0);
	inputLayerNames.emplace_back(inputLayerName1);
	std::vector<std::string> outputLayerNames;
	outputLayerNames.emplace_back(outputLayerName1);

	m_impl->m_trtInference = std::make_shared<trtInf::TrtInference>(
	    onnxPath, inputLayerNames, outputLayerNames, inferDataType, calibImgsFolders);

	trtInf::LayerDim32 inputDims = m_impl->m_trtInference->getLayerDim(inputLayerNames.at(0));
	m_kLogger->debug("inputDims0 = {}x{}x{}x{}", inputDims.d[0], inputDims.d[1], inputDims.d[2], inputDims.d[3]);
	m_inputWidth = inputDims.d[inputDims.nbDims-2]; // e.g. 1x3x416x416
	m_inputHeight = inputDims.d[inputDims.nbDims-1]; // e.g. 1x3x416x416

	m_outputlayerSize0 = m_impl->m_trtInference->getLayerSize(outputLayerNames.at(0));
	trtInf::LayerDim32 outputDims = m_impl->m_trtInference->getLayerDim(outputLayerNames.at(0));
	m_kLogger->debug("outputlayerSize0 = {}", m_outputlayerSize0);
	m_kLogger->debug("outputDims0 = {}x{}x{}x{}", outputDims.d[0], outputDims.d[1], outputDims.d[2], outputDims.d[3]);
	m_outputWidth = outputDims.d[outputDims.nbDims - 2]; // e.g. 1x1x416x416
	m_outputHeight = outputDims.d[outputDims.nbDims - 1]; // e.g. 1x1x416x416
}

StereoDepth::~StereoDepth() {}

void StereoDepth::preProcessing(const cv::Mat& src,
                                cv::Mat& dst,  // const cv::Rect &cropRect,
                                const cv::Mat& mapX,
                                const cv::Mat& mapY,
                                const cv::Size& resize)
{
	// cropping to desired crop size (e.g. 1024x1024)
	// cv::Mat cropedImg = src(cropRect);
	// resize to desired size eg.g 512x512
	cv::Mat resizedImg;
	cv::resize(src, resizedImg, resize);

	// in case we need a gray scale input image
	if (resizedImg.channels() != 1)
	{
		cv::cvtColor(resizedImg, resizedImg, cv::COLOR_BGR2GRAY);
		cv::cvtColor(resizedImg, resizedImg, cv::COLOR_GRAY2BGR);
	}

	// apply remap according to map_x & map_y to rectifiy and undistort image
	cv::remap(resizedImg, dst, mapX, mapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
}

void StereoDepth::postProcessing(const cv::Mat& disparityImg, cv::Mat& dst, const cv::Mat& Q, const cv::Mat& invrectL)
{
	cv::Mat _3dImage;

	CV_Assert(invrectL.size() == Q.size() && "invrectL * Q not same size!");

	// reproject image to 3d
	cv::reprojectImageTo3D(disparityImg, _3dImage, invrectL * Q, true, -1);
	_3dImage = 0.001 * _3dImage;

	// get image with only z information
	std::vector<cv::Mat> channels;
	cv::split(_3dImage, channels);
	dst = channels.at(2);  // last channel is z information

	cv::remap(dst,
	          dst,
	          m_stereoMaps.stereoMapL_inv_x,
	          m_stereoMaps.stereoMapL_inv_y,
	          cv::INTER_LINEAR,
	          cv::BORDER_CONSTANT,
	          0);
	cv::resize(dst, dst, cv::Size(1024, 1024), cv::INTER_NEAREST);
}

cv::Mat StereoDepth::run(const cv::Mat& srcLeft, const cv::Mat& srcRight)
{
	cv::Mat preprocessedLeft;
	preProcessing(srcLeft,
	              preprocessedLeft,
	              m_stereoMaps.stereoMapL_x,
	              m_stereoMaps.stereoMapL_y,
	              cv::Size(m_inputWidth, m_inputHeight));
	cv::Mat preprocessedRight;
	preProcessing(srcRight,
	              preprocessedRight,
	              m_stereoMaps.stereoMapR_x,
	              m_stereoMaps.stereoMapR_y,
	              cv::Size(m_inputWidth, m_inputHeight));

	bool swapRB = true;
	bool crop = false;
	// double scaling = 1.0 / 255.0;
	double scaling = 1.0;

	cv::Scalar mean = cv::Scalar(0, 0, 0);
	cv::Mat blobLeft;
	cv::dnn::blobFromImage(
	    preprocessedLeft, blobLeft, scaling, cv::Size(m_inputWidth, m_inputHeight), mean, swapRB, crop);
	cv::Mat blobRight;
	cv::dnn::blobFromImage(
	    preprocessedRight, blobRight, scaling, cv::Size(m_inputWidth, m_inputHeight), mean, swapRB, crop);

	std::shared_ptr<float[]> out0 = std::shared_ptr<float[]>(new float[m_outputlayerSize0]);
	std::vector<std::shared_ptr<float[]>> outputs = {out0};

	std::vector<trtInf::InputData> blobs;
	trtInf::InputData inputData0;
	inputData0.img = blobLeft;
	inputData0.inputLayerSize = blobLeft.total();
	inputData0.inputIdx = 0;
	blobs.emplace_back(inputData0);
	trtInf::InputData inputData1;
	inputData1.img = blobRight;
	inputData1.inputLayerSize = blobRight.total();
	inputData1.inputIdx = 1;
	blobs.emplace_back(inputData1);

	double dNetwork = (double)cv::getTickCount();
	m_impl->m_trtInference->run(blobs, outputs);  // inference
	// m_kLogger->debug("stereo depth m_trtInference.run() step took ms = {}",
	//                 (((double)cv::getTickCount() - dNetwork) / cv::getTickFrequency() * 1000));

	cv::Mat disparity = cv::Mat(m_outputHeight, m_outputWidth, CV_32FC1, outputs.at(0).get());

	disparity = cv::max(disparity, 0.0);  // clamp all negative values to 0
	cv::Mat afterPostProcess;
	cv::Mat invrectL = m_stereoMaps.rectL.inv();
	postProcessing(disparity, afterPostProcess, m_stereoMaps.Q, invrectL);
	// m_kLogger->debug("m_trtInference & postprocessing step took ms = {}",
	//                 (((double)cv::getTickCount() - dNetwork) / cv::getTickFrequency() * 1000));
	return afterPostProcess;
}
