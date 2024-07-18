/**
 * @copyright Copyright (c) 2020 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * @brief Test for Yolo FilterBySize
 */
#include "yolo_filterBySize_test.h"

#include "projectPaths.h"

YoloFilterBySizeTest::YoloFilterBySizeTest()
{
	fs::path networksConfigPath =
	    utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/yolo_filterBySize/networksConfig.json";

	m_yolo = std::make_shared<yolo::Yolo>(networksConfigPath);
}

// test empty input img
TEST_F(YoloFilterBySizeTest, emptyInput)
{
	cv::Mat img = cv::imread("/home/cobot/Downloads/00_unitTestPipeline/testfiles/yolo_filterBySize/img.png");
	cv::Mat debugImg = img.clone();
	std::shared_ptr<yolo::RectsAndClasses> detections = m_yolo->run(img, debugImg);

	EXPECT_EQ(detections->size(), 1);
}

// ToDo: test memory leakage

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
