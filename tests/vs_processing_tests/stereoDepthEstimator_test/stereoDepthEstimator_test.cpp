/**
 * @copyright Copyright (c) 2020 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * @brief Test for stereoDepthEstimator
 */

#include "stereoDepthEstimator_test.h"
#include "cameraInterface.hpp"

#include "projectPaths.h"

StereoDepthEstimatorTest::StereoDepthEstimatorTest()
{
	fs::path networksConfigPath =
	    utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/stereoDepth/networksConfig.json";
	m_depthEstimator = std::make_shared<StereoDepth>(networksConfigPath, 0);
}

// test all public member functions in a row (normal case)
TEST_F(StereoDepthEstimatorTest, getDepthImg)
{
	VsFrame frame;
	frame.colorImage = cv::imread(utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/stereoDepth/srcLeft.png");
	cv::Mat srcRight = cv::imread(utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/stereoDepth/srcLeft.png");
	frame.depthImage = m_depthEstimator->run(frame.colorImage, srcRight);

	EXPECT_EQ(frame.colorImage.cols, frame.depthImage.cols);
	EXPECT_EQ(frame.colorImage.rows, frame.depthImage.rows);
	EXPECT_EQ(frame.depthImage.type(), CV_32FC1);
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
