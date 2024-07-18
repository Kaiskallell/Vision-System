/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "handEyeAlgo3dBoard_test.h"

#include "handEyeAlgo.h"

HandEyeCalibAlgo3dBoardTest::HandEyeCalibAlgo3dBoardTest() {
const fs::path cameraConfigPath =
	    utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/handEyeAlgo3dBoard/pickCamera.json";
	const fs::path imgDir =
	    utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/handEyeAlgo3dBoard/handEyeCalibImgs";
	const fs::path networksConfigPath =
	    utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/handEyeAlgo3dBoard/handEyeCalibration.json";
	std::vector<VSPose> poses;
	const int camIdxNetConfig = 0;

	nlohmann::json jsonFile = utils::loadJsonFile(networksConfigPath);
	m_poseImage = jsonFile.at("calibrationConfigs").at("ImagePath").get<fs::path>();
	m_measuredPose.x = jsonFile.at("calibrationConfigs")
	                     .at("poses")
	                     .at(camIdxNetConfig)
	                     .at("calibPoses")
	                     .at(0)
	                     .at("coord_x")
	                     .get<float>();
	m_measuredPose.y = jsonFile.at("calibrationConfigs")
	                     .at("poses")
	                     .at(camIdxNetConfig)
	                     .at("calibPoses")
	                     .at(0)
	                     .at("coord_y")
	                     .get<float>();
	m_measuredPose.z = jsonFile.at("calibrationConfigs")
	                     .at("poses")
	                     .at(camIdxNetConfig)
	                     .at("calibPoses")
	                     .at(0)
	                     .at("coord_z")
	                     .get<float>();
	m_measuredPose.roll = jsonFile.at("calibrationConfigs")
	                        .at("poses")
	                        .at(camIdxNetConfig)
	                        .at("calibPoses")
	                        .at(0)
	                        .at("roll")
	                        .get<float>();
	m_measuredPose.pitch = jsonFile.at("calibrationConfigs")
	                         .at("poses")
	                         .at(camIdxNetConfig)
	                         .at("calibPoses")
	                         .at(0)
	                         .at("pitch")
	                         .get<float>();
	m_measuredPose.yaw =
	    jsonFile.at("calibrationConfigs").at("poses").at(camIdxNetConfig).at("calibPoses").at(0).at("yaw").get<float>();
	

	jsonFile = utils::loadJsonFile(cameraConfigPath);
	m_intrinsics.at<double>(0, 0) = jsonFile.at("camera").at(camIdxNetConfig).at("fx").get<double>();
	m_intrinsics.at<double>(1, 1) = jsonFile.at("camera").at(camIdxNetConfig).at("fy").get<double>();
	m_intrinsics.at<double>(0, 2) = jsonFile.at("camera").at(camIdxNetConfig).at("cx").get<double>();
	m_intrinsics.at<double>(1, 2) = jsonFile.at("camera").at(camIdxNetConfig).at("cy").get<double>();
	m_distCoeffs.at<double>(0, 0) = jsonFile.at("camera").at(camIdxNetConfig).at("distCoeffs").at(0).get<double>();
	m_distCoeffs.at<double>(0, 1) = jsonFile.at("camera").at(camIdxNetConfig).at("distCoeffs").at(1).get<double>();
	m_distCoeffs.at<double>(0, 2) = jsonFile.at("camera").at(camIdxNetConfig).at("distCoeffs").at(2).get<double>();
	m_distCoeffs.at<double>(0, 3) = jsonFile.at("camera").at(camIdxNetConfig).at("distCoeffs").at(3).get<double>();
	m_distCoeffs.at<double>(0, 4) = jsonFile.at("camera").at(camIdxNetConfig).at("distCoeffs").at(4).get<double>();
    m_extrinsicsCorrect.at<double>(0, 3) = jsonFile.at("camera").at(camIdxNetConfig).at("x_offset").get<double>();
    m_extrinsicsCorrect.at<double>(1, 3) = jsonFile.at("camera").at(camIdxNetConfig).at("y_offset").get<double>();
    m_extrinsicsCorrect.at<double>(2, 3) = jsonFile.at("camera").at(camIdxNetConfig).at("z_offset").get<double>();

}

TEST_F(HandEyeCalibAlgo3dBoardTest, checkTranslation)
{
	
	
	HandEyeAlgo handEye;
	handEye.cvtLandRUnitToSiUnit(m_measuredPose);

	cv::Mat extrinsics;
	HandEyeAlgo handeyealgo3dBoard;

	cv::Mat measuredPoseImg = cv::imread(m_poseImage);
	handeyealgo3dBoard.calcExtrinsics(m_intrinsics,
	                                  m_distCoeffs,
	                                  measuredPoseImg,
	                                  m_measuredPose,
	                                  extrinsics);  // does only need one measured pose

	double offsetX = extrinsics.at<double>(0, 3);
	double offsetY = extrinsics.at<double>(1, 3);
	double offsetZ = extrinsics.at<double>(2, 3);

	EXPECT_NEAR(offsetX, m_extrinsicsCorrect.at<double>(0, 3), 0.001);
	EXPECT_NEAR(offsetY, m_extrinsicsCorrect.at<double>(1, 3), 0.001);
	EXPECT_NEAR(offsetZ, m_extrinsicsCorrect.at<double>(2, 3), 0.001);
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
