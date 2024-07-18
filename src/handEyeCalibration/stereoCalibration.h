/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef STEREOCALIBRATION_H
#define STEREOCALIBRATION_H

#include <opencv2/opencv.hpp>

#include "projectPaths.h"

#pragma once

class StereoCalibration
{
  public:
	explicit StereoCalibration(const fs::path& networksConfigPath, const fs::path& cameraConfigPath);
	~StereoCalibration();

	void calibrateStereo3D();

  private:
	void detectCalibrationBoard(const cv::Mat& img,
	                            std::vector<int>& ids,
	                            std::vector<std::vector<cv::Point2f>>& corners);

	std::vector<std::vector<cv::Point3f>> generateObjectPoints(const std::vector<int>& ids,
	                                                           const std::string& calibrationboardPath);

	fs::path m_calibrationBoardPath = "";
	cv::Size m_initSize;
	cv::Size m_targetSize;
	size_t m_uOffset;
	size_t m_vOffset;
	cv::Mat m_imgL;
	cv::Mat m_imgR;
	cv::Mat m_intrinsicL;
	cv::Mat m_distCoeffsL;
	cv::Mat m_intrinsicR;
	cv::Mat m_distCoeffsR;
	std::string m_outputFileName = "";
};

#endif