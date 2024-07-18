/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * @brief Test for cpp-deeplabcut
 */

#include "deeplabcut_test.h"

#include "projectPaths.h"

InferDLCTest::InferDLCTest()
{
	fs::path networksConfigPath =
	    utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/dlc/networksConfig.json";
	constexpr size_t dlcIdx = 0;
	m_dlc = std::make_shared<dlc::InferDLC>(networksConfigPath, dlcIdx);
}

TEST_F(InferDLCTest, getDetections)
{
	cv::Mat img = cv::imread(utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/dlc/dlcInput1.png");
	cv::RotatedRect box = cv::RotatedRect(cv::Point2f(63, 63), cv::Size2f(126, 126), 0.0);
	dlc::PointsAndConfs pointsAndConfs = m_dlc->run(img,box);

	std::cout << "pointsAndConfs.keypoints.size() = " << pointsAndConfs.keypoints.size() << std::endl;
	for (int i = 0; i < pointsAndConfs.keypoints.size(); ++i)
	{
		cv::circle(img, pointsAndConfs.keypoints.at(i), 3, cv::Scalar(255, 255, 50), 5);
	}
	// cv::imshow("test", img);
	// cv::waitKey(0);

	EXPECT_EQ(pointsAndConfs.keypoints.size(), 25);
}

TEST_F(InferDLCTest, emptyImg)
{
	cv::Mat img = cv::Mat::zeros(cv::Size(128, 128), CV_8UC3);
	cv::RotatedRect box = cv::RotatedRect(cv::Point2f(63, 63), cv::Size2f(126, 126), 0.0);
	dlc::PointsAndConfs pointsAndConfs = m_dlc->run(img, box);

	EXPECT_EQ(pointsAndConfs.keypoints.empty(), true);
}

TEST_F(InferDLCTest, compareDetectionPoints)
{
	const fs::path expectedKeyPointsPath =
	    utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/dlc/expectedValues";

	cv::Mat img = cv::imread(utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/dlc/dlcInput1.png");
	cv::RotatedRect box = cv::RotatedRect(cv::Point2f(63, 63), cv::Size2f(126, 126), 0.0);

	dlc::PointsAndConfs pointsAndConfs = m_dlc->run(img, box);

	std::ifstream expectedKeyPointsFile(expectedKeyPointsPath);
	std::vector<cv::Point2f> expectedData;

	std::string line;
	while (std::getline(expectedKeyPointsFile, line))
	{
		std::stringstream ss(line);
		float x, y;
		char comma;
		if (ss >> x >> comma >> y)
		{
			expectedData.push_back(cv::Point2f(x, y));
		}
	}

	expectedKeyPointsFile.close();

	std::cout << "expected dlc output:\n " << expectedData << std::endl;
	std::cout << "received dlc output:\n " << pointsAndConfs.keypoints << std::endl;

	constexpr float kThr = 0.1;
	ASSERT_EQ(expectedData.size(), pointsAndConfs.keypoints.size());
	for (int i = 0; i < pointsAndConfs.keypoints.size(); ++i)
	{
		EXPECT_NEAR(expectedData.at(i).x, pointsAndConfs.keypoints.at(i).x, kThr);
		EXPECT_NEAR(expectedData.at(i).y, pointsAndConfs.keypoints.at(i).y, kThr);
	}
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
