/**
 * @copyright Copyright (c) 2020 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * @brief Test for stereoDepthEstimator
 *
 */
#ifndef STEREO_DEPTH_ESTIMATOR_TEST_H
#define STEREO_DEPTH_ESTIMATOR_TEST_H

#include "gtest/gtest.h"
#include "stereoDepth.h"

class StereoDepthEstimatorTest : public ::testing::Test
{
  protected:
	StereoDepthEstimatorTest();
	std::shared_ptr<StereoDepth> m_depthEstimator;
};

#endif