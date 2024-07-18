/**
 * @copyright Copyright (c) 2020 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * @brief Test for Yolo
 */
#include "yolo_test.h"

#include "projectPaths.h"

YoloTest::YoloTest()
{
	fs::path networksConfigPath =
	    utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/yolo/networksConfig.json";

	m_yolo = std::make_shared<yolo::Yolo>(networksConfigPath);
}

// test empty input img
TEST_F(YoloTest, emptyInput)
{
	cv::Mat img = cv::Mat::zeros(cv::Size(1280, 960), CV_8UC3);
	cv::Mat debugImg = img.clone();
	std::shared_ptr<yolo::RectsAndClasses> detections = m_yolo->run(img, debugImg);

	EXPECT_EQ(detections->size(), 0);
}

// ToDo: test memory leakage

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
