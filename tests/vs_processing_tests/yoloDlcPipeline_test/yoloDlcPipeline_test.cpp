/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 */

#include "yoloDLCPipeline.h"

#include "json.hpp"
#include "projectPaths.h"
#include "yoloDlcPipeline_test.h"

YoloDclPipelineTest::YoloDclPipelineTest() {}

TEST_F(YoloDclPipelineTest, multipleDlcs)
{
	// load networksconfig
	fs::path networksConfigPath =
	    "/home/cobot/Downloads/00_unitTestPipeline/testfiles/yoloDlcPipeline/networksConfigArea1.json";
	nlohmann::json jsonFile = utils::loadJsonFile(networksConfigPath);
	// create yoloDlcPipeline object
	YoloDLCPipeline yoloDlcPipeline(networksConfigPath);
	// load img
	cv::Mat img = cv::imread("/home/cobot/Downloads/00_unitTestPipeline/testfiles/yoloDlcPipeline/sourceImage1.png");
	const size_t nMaxResults = 7;
	// inference with yoloDlcPipeline
	YoloDLCResults rectsAndClasses = yoloDlcPipeline.run(img, nMaxResults, YoloDLC::kAllClasses);
	EXPECT_EQ(rectsAndClasses.m_keyPoints.size(), 5);  // all detections(5 rects) need to have keypoints
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
