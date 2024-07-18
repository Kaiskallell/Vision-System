/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "mockCamera.h"

#include <algorithm>
#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "json.hpp"
#include "projectPaths.h"
#include "vs_image.hpp"

bool sortFunction(std::string a, std::string b)
{
	if (a.length() < b.length())
	{
		return true;
	}
	else if (a.length() > b.length())
	{
		return false;
	}
	else
	{
		return a < b;
	}
}

MockCamera::MockCamera(const fs::path& cameraConfigPath,
                           const fs::path& networksConfigPath,
                           const std::string& serialNumber)
{
	// read intrinsic paramters from camera.json
	int camIdxCamConf = utils::getCamIdxFromCameraConfig(serialNumber, cameraConfigPath);

	nlohmann::json jsonFile = utils::loadJsonFile(cameraConfigPath);
	nlohmann::json jsonFileNetwork = utils::loadJsonFile(networksConfigPath);
	this->m_cameraMatrix.fx = jsonFile.at("camera")[camIdxCamConf].at("fx").get<float>();
	this->m_cameraMatrix.fy = jsonFile.at("camera")[camIdxCamConf].at("fy").get<float>();
	this->m_cameraMatrix.cx = jsonFile.at("camera")[camIdxCamConf].at("cx").get<float>();
	this->m_cameraMatrix.cy = jsonFile.at("camera")[camIdxCamConf].at("cy").get<float>();

	m_cropConfigs.cropX = jsonFileNetwork.at("cameraConfigs").at("camera")[0].at("cropX").get<size_t>();
	m_cropConfigs.cropY = jsonFileNetwork.at("cameraConfigs").at("camera")[0].at("cropY").get<size_t>();
	m_cropConfigs.cropWidth = jsonFileNetwork.at("cameraConfigs").at("camera")[0].at("cropWidth").get<size_t>();
	m_cropConfigs.cropHeight = jsonFileNetwork.at("cameraConfigs").at("camera")[0].at("cropHeight").get<size_t>();


	this->m_extrinsics = cv::Mat_<double>(3, 4);
	this->m_extrinsics.at<double>(0, 0) = jsonFile.at("camera")[camIdxCamConf].at("r_11").get<double>();
	this->m_extrinsics.at<double>(0, 1) = jsonFile.at("camera")[camIdxCamConf].at("r_12").get<double>();
	this->m_extrinsics.at<double>(0, 2) = jsonFile.at("camera")[camIdxCamConf].at("r_13").get<double>();
	this->m_extrinsics.at<double>(1, 0) = jsonFile.at("camera")[camIdxCamConf].at("r_21").get<double>();
	this->m_extrinsics.at<double>(1, 1) = jsonFile.at("camera")[camIdxCamConf].at("r_22").get<double>();
	this->m_extrinsics.at<double>(1, 2) = jsonFile.at("camera")[camIdxCamConf].at("r_23").get<double>();
	this->m_extrinsics.at<double>(2, 0) = jsonFile.at("camera")[camIdxCamConf].at("r_31").get<double>();
	this->m_extrinsics.at<double>(2, 1) = jsonFile.at("camera")[camIdxCamConf].at("r_32").get<double>();
	this->m_extrinsics.at<double>(2, 2) = jsonFile.at("camera")[camIdxCamConf].at("r_33").get<double>();

	// offsets
	this->m_extrinsics.at<double>(0, 3) = jsonFile.at("camera")[camIdxCamConf].at("x_offset").get<double>();
	this->m_extrinsics.at<double>(1, 3) = jsonFile.at("camera")[camIdxCamConf].at("y_offset").get<double>();
	this->m_extrinsics.at<double>(2, 3) = jsonFile.at("camera")[camIdxCamConf].at("z_offset").get<double>();
	this->m_serialNumber = serialNumber;
	
	m_kLogger->debug("correcting intrinsics according to crop of img");
	this->m_cameraMatrix.cx = this->m_cameraMatrix.cx - m_cropConfigs.cropX;
	this->m_cameraMatrix.cy = this->m_cameraMatrix.cy - m_cropConfigs.cropY;

	int camIdxNetConf = utils::getCamIdxFromNetworksConfig(serialNumber, networksConfigPath);
	jsonFile = utils::loadJsonFile(networksConfigPath);
	std::experimental::filesystem::path path_root =
	    jsonFile.at("cameraConfigs").at("camera")[camIdxNetConf].at("mockImgsPath").get<std::string>();

	// check if it is a stereoCam
	if (jsonFile.at("cameraConfigs").at("camera")[camIdxNetConf].at("serialNumbers").size() == 2)
	{
		// change path to imgs according to serialNumber (order in networksconfig file)
		if (jsonFile.at("cameraConfigs").at("camera")[camIdxNetConf].at("serialNumbers").at(0) == serialNumber)
		{
			path_root = path_root / "cam0";
		}
		if (jsonFile.at("cameraConfigs").at("camera")[camIdxNetConf].at("serialNumbers").at(1) == serialNumber)
		{
			path_root = path_root / "cam1";
		}
	}

	fs::path colorImagesPath = path_root / "colorImages";

	std::vector<std::string> colorImagePaths = getFilenames(colorImagesPath);

	m_kLogger->info("Loading color images from {}", colorImagesPath);

	for (int i = 0; i < colorImagePaths.size(); ++i)
	{
		m_imagesPaths.push_back({colorImagePaths.at(i)});
	}

	current_image = m_imagesPaths.begin();

	cv::Mat img = cv::imread(current_image->color_image_path);

	m_imgWidth = img.size().width;
	m_imgHeight = img.size().height;

	m_frame.colorImage = cv::Mat::zeros(img.size(), CV_8UC3);
}

MockCamera::~MockCamera() {}

VsFrame MockCamera::vsGetFrame()
{
	m_kLogger->debug("Color image path: {}", current_image->color_image_path);
	m_frame.colorImage = cv::imread(current_image->color_image_path);
	m_frame.secondBgrImage = cv::Mat();  // empty img

	if (++current_image == m_imagesPaths.end())
	{
		// no more images, loop around?
		current_image = m_imagesPaths.begin();
	}

	return m_frame;
}

bool MockCamera::init(TriggerMode triggerMode) { return true; }

int MockCamera::close() { return 0; }

std::vector<std::string> MockCamera::getFilenames(std::experimental::filesystem::path path)
{
	namespace stdfs = std::experimental::filesystem;

	std::vector<std::string> filenames;

	const stdfs::directory_iterator end{};

	for (stdfs::directory_iterator iter{path}; iter != end; ++iter)
	{
		if (stdfs::is_regular_file(*iter))
		{
			filenames.push_back(iter->path().string());
		}
	}

	std::sort(filenames.begin(), filenames.end(), sortFunction);

	return filenames;
}

VsCameraIntrinsics MockCamera::vsGetCameraMatrix() { return this->m_cameraMatrix; }

cv::Mat_<double> MockCamera::getExtrinsics() { return this->m_extrinsics; }

size_t MockCamera::getImgWidth() { return m_imgWidth; }

size_t MockCamera::getImgHeight() { return m_imgHeight; }
