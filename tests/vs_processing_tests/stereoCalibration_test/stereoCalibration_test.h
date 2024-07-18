/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef STEREO_CALIBRATION_TEST_H
#define STEREO_CALIBRATION_TEST_H

#include <opencv2/opencv.hpp>

#include "gtest/gtest.h"

class StereoCalibrationTest : public ::testing::Test
{
  protected:
	StereoCalibrationTest();
	void expectMatSimilar(const cv::Mat& expected, const cv::Mat& actual, double equalityPercentage);
};

#endif