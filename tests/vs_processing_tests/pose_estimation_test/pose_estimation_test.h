/**
 * @copyright Copyright (c) 2020 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * @brief Test for Pose Estimation with 3d camera
 *
 */
#ifndef POSE_ESTIMATION_TEST_H
#define POSE_ESTIMATION_TEST_H

#include <opencv2/opencv.hpp>

#include "cameraInterface.hpp"
#include "gtest/gtest.h"
#include "vs_poseObject.h"

class PoseEstimationTest : public ::testing::Test
{
  protected:
	PoseEstimationTest();

	cv::Mat_<double> m_extrinsics = cv::Mat_<double>(3, 4);
	VsCameraIntrinsics m_intrinsics;
	cv::Mat m_mask;
	cv::Mat_<float> m_depth_img;
	std::vector<cv::Point2f> m_keyPoints;
	VSPose m_desiredPose;
};

#endif