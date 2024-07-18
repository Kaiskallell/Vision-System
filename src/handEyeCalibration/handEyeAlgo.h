/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef HANDEYEALGO_H
#define HANDEYEALGO_H

#pragma once

#include "logging/logging.h"
#include "opencv2/opencv.hpp"
#include "projectPaths.h"
#include "vs_poseObject.h"

class HandEyeAlgo
{
  public:
	HandEyeAlgo();
	~HandEyeAlgo();
	void calcIntrinsics(const fs::path& imgFolderPath, cv::Mat& intrinsics, cv::Mat& distCoeffs);
	void calcExtrinsics(const cv::Mat& intrinsics,
	                    const cv::Mat& distCoeffs,
	                    const cv::Mat& measuredPoseImg,
	                    const VSPose& measuredPose,
	                    cv::Mat& extrinsics);
	void cvtLandRUnitToSiUnit(VSPose& pose);

  private:
	void detectCalibrationBoard(const cv::Mat& img, std::vector<int>& ids, std::vector<cv::Point2f>& flatCorners);
	std::vector<cv::Point3f> generateObjectPoints(const std::vector<int>& ids,
	                                              const std::string& calibrationboard_path);
	cv::Mat eulerAnglesToRotationMatrix(double roll, double pitch, double yaw);
	void estimatePoseCalibrationBoard(const std::vector<int>& ids,
	                                  const std::vector<cv::Point2f>& corners,
	                                  const cv::Mat& cameraMatrix,
	                                  const cv::Mat& distCoeffs,
	                                  cv::Mat& rvec,
	                                  cv::Mat& tvec);
	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("HandEyeAlgo3dBoard");
};

#endif