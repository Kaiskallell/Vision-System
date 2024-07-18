/**
 * @copyright Copyright (c) 2021 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * @brief Test for Algorithm for hand eye calibration
 *
 */
#ifndef HAND_EYE_CALIBRATION_ALGO_TEST_H
#define HAND_EYE_CALIBRATION_ALGO_TEST_H

#include "projectPaths.h"
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include "vs_poseObject.h"
#include "gtest/gtest.h"

class HandEyeCalibAlgo3dBoardTest : public ::testing::Test
{
  protected:
	HandEyeCalibAlgo3dBoardTest();
  VSPose m_measuredPose;
  cv::Mat m_extrinsicsCorrect = cv::Mat::eye(cv::Size(3, 3), CV_64FC1);
  cv::Mat m_intrinsics = cv::Mat::eye(cv::Size(3, 3), CV_64FC1);
  cv::Mat m_distCoeffs = cv::Mat::zeros(cv::Size(5, 1), CV_64FC1);
  fs::path m_poseImage;
};

#endif
