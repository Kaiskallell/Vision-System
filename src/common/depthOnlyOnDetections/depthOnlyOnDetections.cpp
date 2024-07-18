/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "depthOnlyOnDetections.h"

#include "opencv2/cudaarithm.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudastereo.hpp"
#include "opencv2/cudawarping.hpp"
#include "trtInference.h"
class DepthOnlyOnDetections::Impl
{
  public:
	Impl() = default;
	~Impl() = default;
	std::shared_ptr<trtInf::TrtInference> m_trtInference;
};

StereoMaps2 loadStereoCalibParams(const fs::path& paramPath)
{
	// load xml file
	cv::FileStorage fileStorage(paramPath.string(), cv::FileStorage::READ);

	StereoMaps2 stereoMaps;
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

	// only needed for  rectify img with cuda support
	cv::Mat mapXFloat;
	cv::Mat mapYFloat;
	cv::convertMaps(stereoMaps.stereoMapL_x, stereoMaps.stereoMapL_y, mapXFloat, mapYFloat, CV_32FC1);

	stereoMaps.gpuMatMapX.upload(mapXFloat);
	stereoMaps.gpuMatMapY.upload(mapYFloat);

	// for reprojectto3d
	stereoMaps.reprojectQ = stereoMaps.rectL.inv() * stereoMaps.Q;

	// for reprojectto3d cuda version
	stereoMaps.reprojectQ.convertTo(stereoMaps.reprojectQFp32, CV_32F);

	cv::Mat mapXInvFloat;
	cv::Mat mapYInvFloat;
	cv::convertMaps(stereoMaps.stereoMapL_inv_x, stereoMaps.stereoMapL_inv_y, mapXInvFloat, mapYInvFloat, CV_32FC1);
	stereoMaps.stereoMapL_inv_x_gpu.upload(mapXInvFloat);
	stereoMaps.stereoMapL_inv_y_gpu.upload(mapYInvFloat);
	return stereoMaps;
}

DepthOnlyOnDetections::DepthOnlyOnDetections(const fs::path& networksConfigPath)
{
	m_impl = std::make_unique<Impl>();

	std::string networkType = "stereoDepth";
	nlohmann::json jsonFile = utils::loadJsonFile(networksConfigPath);

	// assign default values
	fs::path stereoMatFilePath = "";
	std::string inputLayerName0 = "";  // needs to be "left"
	std::string inputLayerName1 = "";
	std::string outputLayerName1 = "";
	fs::path onnxPath = "";
	std::string inferDataType = "";
	fs::path calibImgsPath0 = "";
	fs::path calibImgsPath1 = "";
	try
	{
		stereoMatFilePath = jsonFile.at(networkType).at("mappingConfigPath").get<std::string>();
		inputLayerName0 =
			jsonFile.at(networkType).at("inputLayerName0").get<std::string>();  // needs to be "left"
		inputLayerName1 =
			jsonFile.at(networkType).at("inputLayerName1").get<std::string>();  // needs to be "right"
		outputLayerName1 =
			jsonFile.at(networkType).at("outputLayerName1").get<std::string>();  // needs to be "output"
		onnxPath = jsonFile.at(networkType).at("onnxPath").get<std::string>();
		inferDataType = jsonFile.at(networkType).at("dataType").get<std::string>();
		calibImgsPath0 = jsonFile.at(networkType).at("calibImgsPath0").get<std::string>();
		calibImgsPath1 = jsonFile.at(networkType).at("calibImgsPath1").get<std::string>();
	}
	catch (const std::exception& e)
	{
		m_kLogger->error("Could not read parameters from file: " + networksConfigPath.string());
	}
	
	
	const std::vector<fs::path> calibImgsFolders = {calibImgsPath0, calibImgsPath1};

	m_stereoMaps = loadStereoCalibParams(stereoMatFilePath);

	std::vector<std::string> inputLayerNames;
	inputLayerNames.emplace_back(inputLayerName0);
	inputLayerNames.emplace_back(inputLayerName1);
	std::vector<std::string> outputLayerNames;
	outputLayerNames.emplace_back(outputLayerName1);

	m_impl->m_trtInference =
	    std::make_shared<trtInf::TrtInference>(onnxPath, inputLayerNames, outputLayerNames, inferDataType);

	trtInf::LayerDim32 inputDims = m_impl->m_trtInference->getLayerDim(inputLayerNames.at(0));
	m_kLogger->debug("inputDims0 = {}x{}x{}x{}", inputDims.d[0], inputDims.d[1], inputDims.d[2], inputDims.d[3]);
	trtInf::LayerDim32 outputDims = m_impl->m_trtInference->getLayerDim(outputLayerNames.at(0));
	m_kLogger->debug("outputDims0 = {}x{}x{}x{}", outputDims.d[0], outputDims.d[1], outputDims.d[2], outputDims.d[3]);

	m_batchSize = inputDims.d[0];            // e.g. 5
	m_inputLayerDimSize = inputDims.d[2];    // e.g. 320
	m_outputLayerDimSize = outputDims.d[2];  // e.g. 320
	m_windowSize = 416;

	m_outputlayerSize0 = m_impl->m_trtInference->getLayerSize(outputLayerNames.at(0));
	m_kLogger->debug("m_outputlayerSize0 = {}", m_outputlayerSize0);
}

DepthOnlyOnDetections::~DepthOnlyOnDetections() {}

template <typename T>
std::vector<size_t> sortIndexes(const std::vector<T>& v)
{
	std::vector<size_t> idx(v.size());
	std::iota(idx.begin(), idx.end(), 0);
	std::sort(idx.begin(), idx.end(), [](const T& lhs, const T& rhs) { return lhs < rhs; });
	return idx;
}

cv::Mat postProcessing(const size_t inputLayerSize,
                       const size_t windowSize,
                       const std::vector<cv::Point>& upperRightCorners,
                       const cv::Mat& imgL_rectified,
                       const StereoMaps2& stereoMaps,
                       const std::vector<cv::Mat>& disparities)
{
	double scalingFactor = static_cast<double>(inputLayerSize) / windowSize;

	// resize the disparities back to bigger resolution from 320x320 to 416x414
	std::vector<cv::Mat> disparitiesPost;
	for (const auto& disp : disparities)
	{
		cv::Mat dispScaled = disp / scalingFactor;
		cv::resize(dispScaled, dispScaled, cv::Size(), 1 / scalingFactor, 1 / scalingFactor, cv::INTER_LINEAR);
		disparitiesPost.push_back(dispScaled);
	}

	cv::Mat fullDisparity = cv::Mat::zeros(imgL_rectified.size(), CV_32FC1);  // 1024x1024
	// metric for priorizing which disparity should be used first to reconstuct the full disparity img
	std::vector<double> upperRightCornersDistances;
	for (const auto& corner : upperRightCorners)
	{
		double distance = std::pow(corner.x - imgL_rectified.cols, 2) + std::pow(corner.y, 2);
		upperRightCornersDistances.push_back(distance);
	}
	std::vector<size_t> upperRightCornersIndicesSorted = sortIndexes(upperRightCornersDistances);

	// constructing the full disparity img from multiple small disparity imgs
	for (const auto& i : upperRightCornersIndicesSorted)
	{
		cv::Point upperRightCorner = upperRightCorners[i];
		cv::Mat disparityPost = disparitiesPost[i];
		int d_w = disparityPost.cols;
		int d_h = disparityPost.rows;

		if (upperRightCorner.x - d_w <= 0)
		{
			d_w = upperRightCorner.x;
		}
		if (upperRightCorner.y + d_h >= imgL_rectified.rows)
		{
			d_h = imgL_rectified.rows - upperRightCorner.y;
		}

		disparityPost(cv::Rect(disparityPost.cols - d_w, 0, d_w, d_h))
		    .copyTo(fullDisparity(cv::Rect(upperRightCorner.x - d_w, upperRightCorner.y, d_w, d_h)));
	}

	cv::Mat depth = cv::Mat::zeros(fullDisparity.size(), CV_32FC1);

	cv::Mat _3dImage;
	// reproject image to 3d
	cv::reprojectImageTo3D(fullDisparity, _3dImage, stereoMaps.reprojectQ, true, -1);
	_3dImage = 0.001 * _3dImage;

	// get image with only z information
	std::vector<cv::Mat> channels;
	cv::split(_3dImage, channels);
	depth = channels.at(2);  // last channel is z information
	// convert from rectified to unrectified coordinate system
	cv::remap(depth,
	          depth,
	          stereoMaps.stereoMapL_inv_x,
	          stereoMaps.stereoMapL_inv_y,
	          cv::INTER_LINEAR,
	          cv::BORDER_CONSTANT,
	          0);
	cv::resize(depth, depth, cv::Size(1024, 1024), cv::INTER_NEAREST);

	return depth;
}

cv::Mat postProcessingCuda(const size_t inputLayerSize,
                           const size_t windowSize,
                           const std::vector<cv::Point>& upperRightCorners,
                           const cv::Mat& imgL_rectified,
                           const StereoMaps2& stereoMaps,
                           const std::vector<cv::Mat>& disparities)
{
	double scalingFactor = static_cast<double>(inputLayerSize) / windowSize;

	// resize the disparities back to bigger resolution from 320x320 to 416x414
	std::vector<cv::Mat> disparitiesPost;
	for (const auto& disp : disparities)
	{
		cv::Mat dispScaled = disp / scalingFactor;
		cv::resize(dispScaled, dispScaled, cv::Size(), 1 / scalingFactor, 1 / scalingFactor, cv::INTER_LINEAR);
		disparitiesPost.push_back(dispScaled);
	}

	cv::Mat fullDisparity = cv::Mat::zeros(imgL_rectified.size(), CV_32FC1);  // 1024x1024
	// metric for priorizing which disparity should be used first to reconstuct the full disparity img
	std::vector<double> upperRightCornersDistances;
	for (const auto& corner : upperRightCorners)
	{
		double distance = std::pow(corner.x - imgL_rectified.cols, 2) + std::pow(corner.y, 2);
		upperRightCornersDistances.push_back(distance);
	}
	std::vector<size_t> upperRightCornersIndicesSorted = sortIndexes(upperRightCornersDistances);

	// constructing the full disparity img from multiple small disparity imgs
	for (const auto& i : upperRightCornersIndicesSorted)
	{
		cv::Point upperRightCorner = upperRightCorners[i];
		cv::Mat disparityPost = disparitiesPost[i];
		int d_w = disparityPost.cols;
		int d_h = disparityPost.rows;

		if (upperRightCorner.x - d_w <= 0)
		{
			d_w = upperRightCorner.x;
		}
		if (upperRightCorner.y + d_h >= imgL_rectified.rows)
		{
			d_h = imgL_rectified.rows - upperRightCorner.y;
		}

		disparityPost(cv::Rect(disparityPost.cols - d_w, 0, d_w, d_h))
		    .copyTo(fullDisparity(cv::Rect(upperRightCorner.x - d_w, upperRightCorner.y, d_w, d_h)));
	}

	cv::cuda::GpuMat depthGPU = cv::cuda::GpuMat(fullDisparity.size(), CV_32FC1);
	double dreporject = (double)cv::getTickCount();
	cv::Mat _3dImage;
	// reproject image to 3d
	cv::cuda::GpuMat fullDisparityGPU;
	fullDisparityGPU.upload(fullDisparity);
	cv::cuda::GpuMat _3dImageGPU;
	constexpr int knChannels = 3;
	cv::cuda::reprojectImageTo3D(fullDisparityGPU, _3dImageGPU, stereoMaps.reprojectQFp32, knChannels);

	cv::cuda::GpuMat scaled3dImageGPU(_3dImageGPU.size(), _3dImageGPU.type());
	_3dImageGPU.convertTo(scaled3dImageGPU, _3dImageGPU.type(), 0.001);
	cv::cuda::threshold(
	    scaled3dImageGPU,
	    scaled3dImageGPU,
	    10.0,
	    10.0,
	    cv::THRESH_TRUNC);  // cuda implementation of reprojectImageTo3d is different from cpu (handling of inf values)
	// get image with only z information
	std::vector<cv::cuda::GpuMat> channelsGPU;
	cv::cuda::split(scaled3dImageGPU, channelsGPU);
	depthGPU = channelsGPU.at(2);  // last channel is z information

	// convert from rectified to unrectified coordinate system
	cv::cuda::GpuMat depthUnrectifiedGpu;
	cv::cuda::remap(depthGPU,
	                depthUnrectifiedGpu,
	                stereoMaps.stereoMapL_inv_x_gpu,
	                stereoMaps.stereoMapL_inv_y_gpu,
	                cv::INTER_LINEAR,
	                cv::BORDER_CONSTANT,
	                0);
	cv::cuda::GpuMat depthUnrectifiedResizedGpu;
	cv::cuda::resize(depthUnrectifiedGpu, depthUnrectifiedResizedGpu, cv::Size(1024, 1024), cv::INTER_NEAREST);

	cv::Mat depth;
	depthUnrectifiedResizedGpu.download(depth);

	return depth;
}

void rectifyImg(const cv::Mat& src, cv::Mat& dst, const cv::Mat& mapX, const cv::Mat& mapY, const cv::Size& resize)
{
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

void rectifyImgCuda(const cv::Mat& src,
                    cv::Mat& dst,
                    const cv::cuda::GpuMat& mapXGpu,
                    const cv::cuda::GpuMat& mapYGpu,
                    const cv::Size& resize)
{
	cv::cuda::GpuMat gpuMatSrc;
	gpuMatSrc.upload(src);

	cv::cuda::GpuMat gpuMatResizedImg;
	cv::cuda::resize(gpuMatSrc, gpuMatResizedImg, resize);

	cv::cuda::GpuMat gpuMatResizedImg3Channel;
	cv::cuda::GpuMat gpuMatResizedImg1Channel;
	// in case we need a gray scale input image
	if (gpuMatResizedImg.channels() != 1)
	{
		cv::cuda::cvtColor(gpuMatResizedImg, gpuMatResizedImg1Channel, cv::COLOR_BGR2GRAY);
		cv::cuda::cvtColor(gpuMatResizedImg1Channel, gpuMatResizedImg3Channel, cv::COLOR_GRAY2BGR);
	}

	// apply remap according to map_x & map_y to rectifiy and undistort image
	cv::cuda::GpuMat gpuMatDst = cv::cuda::GpuMat(mapXGpu.size(), gpuMatResizedImg3Channel.type());

	cv::cuda::remap(gpuMatResizedImg3Channel, gpuMatDst, mapXGpu, mapYGpu, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
	gpuMatDst.download(dst);
}

void DepthOnlyOnDetections::run(const cv::Mat& imgLeft,
                                const cv::Mat& imgRight,
                                const std::vector<cv::Rect>& detectedBBoxes,
                                cv::Mat& dstDepth)
{
	double drectifiy = (double)cv::getTickCount();
	// rectify camera imgs
	cv::Mat imgL_rectified;
	rectifyImg(imgLeft, imgL_rectified, m_stereoMaps.stereoMapL_x, m_stereoMaps.stereoMapL_y, cv::Size(1024, 1024));
	cv::Mat imgR_rectified;
	rectifyImg(imgRight, imgR_rectified, m_stereoMaps.stereoMapR_x, m_stereoMaps.stereoMapR_y, cv::Size(1024, 1024));
	// TODO(aschaefer): cuda version yields slightly different values
	// cv::Mat imgL_rectified;
	// rectifyImgCuda(imgLeft, imgL_rectified, m_stereoMaps.gpuMatMapX, m_stereoMaps.gpuMatMapY, cv::Size(1024, 1024));
	// cv::Mat imgR_rectified;
	// rectifyImgCuda(imgRight, imgR_rectified, m_stereoMaps.gpuMatMapX, m_stereoMaps.gpuMatMapY, cv::Size(1024, 1024));
	// m_kLogger->debug("drectifiy step took ms = {}",
	//                 (((double)cv::getTickCount() - drectifiy) / cv::getTickFrequency() * 1000));

	double drectifiyBBox = (double)cv::getTickCount();

	// rectify the detected bboxes
	std::vector<cv::Rect> bboxesRectified;
	for (const auto& bbox : detectedBBoxes)
	{
		// detected Bounding Box --> corrected Bounding Box
		// convert rect in vector of points
		std::vector<cv::Point> cornersDetected = {{bbox.x, bbox.y},
		                                          {bbox.x + bbox.width, bbox.y},
		                                          {bbox.x + bbox.width, bbox.y + bbox.height},
		                                          {bbox.x, bbox.y + bbox.height}};  // [u,v]
		std::vector<cv::Point> cornersRectified;
		// transform corners in rectified coords
		for (const auto& corner : cornersDetected)
		{
			cv::Point cornerRectified = m_stereoMaps.stereoMapL_inv_x.at<cv::Point_<short>>(corner.y, corner.x);  // u,v
			cornersRectified.push_back(cornerRectified);
		}
		// calc rectified bbox
		cv::Rect bboxRectified = cv::boundingRect(cornersRectified);
		bboxesRectified.push_back(bboxRectified);
	}
	// m_kLogger->debug("drectifiyBBox step took ms = {}",
	//                 (((double)cv::getTickCount() - drectifiyBBox) / cv::getTickFrequency() * 1000));

	double dwindowGeneration = (double)cv::getTickCount();
	std::vector<cv::Mat> productWindowsL;
	std::vector<cv::Mat> productWindowsR;
	for (int i = 0; i < m_batchSize; ++i)
	{
		productWindowsL.emplace_back(cv::Mat::zeros(cv::Size(m_windowSize, m_windowSize), CV_8UC3));
		productWindowsR.emplace_back(cv::Mat::zeros(cv::Size(m_windowSize, m_windowSize), CV_8UC3));
	}

	std::vector<cv::Point> upperRightCorners;
	for (int i = 0; i < std::min(m_batchSize, static_cast<int>(bboxesRectified.size())); i++)
	{
		cv::Rect bbox = bboxesRectified[i];
		cv::Point upperRightCorner(bbox.x + bbox.width, bbox.y);
		upperRightCorners.push_back(upperRightCorner);

		cv::Rect roiR(
		    std::max(upperRightCorner.x - m_windowSize, 0),
		    upperRightCorner.y,
		    std::min(upperRightCorner.x, imgR_rectified.cols) - std::max(upperRightCorner.x - m_windowSize, 0),
		    std::min(upperRightCorner.y + m_windowSize, imgR_rectified.rows) - upperRightCorner.y);
		cv::Rect roiL(
		    std::max(upperRightCorner.x - m_windowSize, 0),
		    upperRightCorner.y,
		    std::min(upperRightCorner.x, imgL_rectified.cols) - std::max(upperRightCorner.x - m_windowSize, 0),
		    std::min(upperRightCorner.y + m_windowSize, imgL_rectified.rows) - upperRightCorner.y);

		cv::Mat productWindowL = imgL_rectified(roiL);
		cv::Mat productWindowR = imgR_rectified(roiR);

		productWindowL.copyTo(productWindowsL.at(i)(
		    cv::Rect(m_windowSize - productWindowL.cols, 0, productWindowL.cols, productWindowL.rows)));
		productWindowR.copyTo(productWindowsR.at(i)(
		    cv::Rect(m_windowSize - productWindowR.cols, 0, productWindowR.cols, productWindowR.rows)));
	}
	// m_kLogger->debug("dwindowGeneration step took ms = {}",
	//                 (((double)cv::getTickCount() - dwindowGeneration) / cv::getTickFrequency() * 1000));

	double dblobGeneration = (double)cv::getTickCount();
	// inference with tensorrt
	bool swapRB = false;
	bool crop = false;
	double scaling = 1.0;

	cv::Scalar mean = cv::Scalar(0, 0, 0);
	cv::Mat blobLeft;
	cv::dnn::blobFromImages(
	    productWindowsL, blobLeft, scaling, cv::Size(m_inputLayerDimSize, m_inputLayerDimSize), mean, swapRB, crop);

	cv::Mat blobRight;
	cv::dnn::blobFromImages(
	    productWindowsR, blobRight, scaling, cv::Size(m_inputLayerDimSize, m_inputLayerDimSize), mean, swapRB, crop);

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
	// m_kLogger->debug("dblobGeneration step took ms = {}",
	//                 (((double)cv::getTickCount() - dblobGeneration) / cv::getTickFrequency() * 1000));

	double dNetwork = (double)cv::getTickCount();
	m_impl->m_trtInference->run(blobs, outputs);  // inference
	// m_kLogger->debug("m_trtInference.run() step took ms = {}",
	//                 (((double)cv::getTickCount() - dNetwork) / cv::getTickFrequency() * 1000));

	double dimgsFromblob = (double)cv::getTickCount();
	int outputSz[] = {m_batchSize, 1, m_outputLayerDimSize, m_outputLayerDimSize};
	cv::Mat disparitiesBlob = cv::Mat(4, outputSz, CV_32F, outputs.at(0).get());
	std::vector<cv::Mat> disparities;
	cv::dnn::imagesFromBlob(disparitiesBlob, disparities);
	// m_kLogger->debug("dimgsFromblob step took ms = {}",
	//                 (((double)cv::getTickCount() - dimgsFromblob) / cv::getTickFrequency() * 1000));

	double dPostProcessing = (double)cv::getTickCount();
	// dstDepth =
	//     postProcessing(m_inputLayerDimSize, m_windowSize, upperRightCorners, imgL_rectified, m_stereoMaps,
	//     disparities);
	dstDepth = postProcessingCuda(
	    m_inputLayerDimSize, m_windowSize, upperRightCorners, imgL_rectified, m_stereoMaps, disparities);
	// m_kLogger->debug("dPostProcessing step took ms = {}",
	//                 (((double)cv::getTickCount() - dPostProcessing) / cv::getTickFrequency() * 1000));
}
