/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 */

#ifndef POSE_ESTIMATION_IPA_H
#define POSE_ESTIMATION_IPA_H

#include <opencv2/opencv.hpp>

#include "cameraInterface.hpp"
#include "logging.h"
#include "projectPaths.h"
#include "vs_poseObject.h"

class PoseEstimationIPA
{
  public:
	PoseEstimationIPA(const fs::path& networksConfigPath);
	~PoseEstimationIPA();
	std::vector<VSPose> getPoses(const VsCameraIntrinsics& intrinsics,
	                             const cv::Mat_<double>& extrinsics,
	                             const cv::Mat_<float>& depthImg,
	                             cv::Mat& debugImg);

  private:
	class Impl;
	std::unique_ptr<Impl> m_impl;
};

#endif