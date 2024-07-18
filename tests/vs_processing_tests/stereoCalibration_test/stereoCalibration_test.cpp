/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "stereoCalibration_test.h"

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>

#include "projectPaths.h"
#include "stereoCalibration.h"

void StereoCalibrationTest::expectMatSimilar(const cv::Mat& expected,
                                             const cv::Mat& actual,
                                             const double equalityPercentage)
{
	ASSERT_EQ(832, expected.rows);
	ASSERT_EQ(832, expected.cols);
	ASSERT_EQ(expected.type(), actual.type());

	constexpr int totalValues = 832 * 832;
	int numMismatchedValues = 0;

	for (int i = 0; i < 832; ++i)
	{
		for (int j = 0; j < 832; ++j)
		{
			int expectedValue = static_cast<int>(expected.at<uchar>(i, j));
			int actualValue = static_cast<int>(actual.at<uchar>(i, j));
			int diff = std::abs(expectedValue - actualValue);
			double similarity = 1.0 - static_cast<double>(diff) / 255.0;
			if (similarity < (equalityPercentage / 100))
			{
				++numMismatchedValues;
			}
		}
	}

	double similarityPercentage = 100.0 * static_cast<double>(totalValues - numMismatchedValues) / totalValues;
	ASSERT_GE(similarityPercentage, equalityPercentage);
}

StereoCalibrationTest::StereoCalibrationTest() {}

TEST_F(StereoCalibrationTest, compareMatrices)
{
	const fs::path cameraConfigPath =
	    utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/StereoCalibration/pickCamera.json";
	const fs::path networksConfigPath =
	    utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/StereoCalibration/stereoCalibration.json";

	StereoCalibration stereoCalib(networksConfigPath, cameraConfigPath);
	stereoCalib.calibrateStereo3D();

	// Determining the actual result values
	fs::path stereoConfigFilePath = utils::getProjectRootDir() / "config" / "stereoMap_inv_calculated.xml";
	cv::FileStorage actualValues(stereoConfigFilePath, cv::FileStorage::READ);
	cv::Mat actual_values = actualValues["stereoMapL_x"].mat();
	actualValues.release();

	// Determining the expected result values
	cv::FileStorage expectedValues(
	    "/home/cobot/Downloads/00_unitTestPipeline/testfiles/StereoCalibration/stereoMap_inv_Expected.xml",
	    cv::FileStorage::READ);
	cv::Mat expected_values = expectedValues["stereoMapL_x"].mat();
	expectedValues.release();

	// Checking the matrices for similarity of results
	constexpr double equalityPercentage = 99.98;
	expectMatSimilar(expected_values, actual_values, equalityPercentage);
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
