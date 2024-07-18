/**
 * @copyright Copyright (c) 2020 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * @brief Test for Pose Estimation
 */

#include "pose_estimation_test.h"

#include "poseEstimation.h"
#include "projectPaths.h"

PoseEstimationTest::PoseEstimationTest()
{
	std::experimental::filesystem::path projctRootDir = utils::getProjectRootDir();
	// read all data needed for poseEstimation class
	std::string inputDataPath =
	    utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/poseEstimation/poseEstimationTestInput.yml";
	cv::FileStorage fs(inputDataPath, cv::FileStorage::READ);

	if (!fs.isOpened())
	{
		std::cerr << "Failed to open " << inputDataPath << std::endl;
		EXPECT_EQ(1, 0);
		return;
	}

	m_intrinsics.fx = fs["fx"];
	m_intrinsics.fy = fs["fy"];
	m_intrinsics.cx = fs["cx"];
	m_intrinsics.cy = fs["cy"];

	fs["maskImg0"] >> m_mask;
	fs["depthImg"] >> m_depth_img;

	cv::FileNode node = fs["keyPoints0"];
	if (node.type() != cv::FileNode::SEQ)
	{
		std::cerr << "keyPoints0 is not a sequence! FAIL" << std::endl;
		EXPECT_EQ(1, 0);
		return;
	}
	cv::FileNodeIterator it = node.begin(), it_end = node.end();  // Go through the node
	for (; it != it_end; ++it)
	{
		cv::Point2f point;
		*it >> point;
		m_keyPoints.push_back(point);
	}

	fs["r_11"] >> m_extrinsics.at<double>(0, 0);
	fs["r_12"] >> m_extrinsics.at<double>(0, 1);
	fs["r_13"] >> m_extrinsics.at<double>(0, 2);
	fs["r_21"] >> m_extrinsics.at<double>(1, 0);
	fs["r_22"] >> m_extrinsics.at<double>(1, 1);
	fs["r_23"] >> m_extrinsics.at<double>(1, 2);
	fs["r_31"] >> m_extrinsics.at<double>(2, 0);
	fs["r_32"] >> m_extrinsics.at<double>(2, 1);
	fs["r_33"] >> m_extrinsics.at<double>(2, 2);

	// offsets
	fs["x_offset"] >> m_extrinsics.at<double>(0, 3);
	fs["y_offset"] >> m_extrinsics.at<double>(1, 3);
	fs["z_offset"] >> m_extrinsics.at<double>(2, 3);

	fs["pX0"] >> m_desiredPose.x;
	fs["pY0"] >> m_desiredPose.y;
	fs["pZ0"] >> m_desiredPose.z;
	fs["pRoll0"] >> m_desiredPose.roll;
	fs["pPitch0"] >> m_desiredPose.pitch;
	fs["pYaw0"] >> m_desiredPose.yaw;
}

// test all public member functions in a row (normal case)
TEST_F(PoseEstimationTest, completePoseEstimation)
{
	VSPose poseCamCoords;

	fs::path networksConfigPath =
	    utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/poseEstimation/networksConfig.json";
	PoseEstimation poseEst(networksConfigPath);
	cv::Mat debugImg;

	double startTime = (double)cv::getTickCount();
	cv::Rect rect = cv::boundingRect(m_keyPoints);
	int classId = 0;

	VSPose poseRobotCoords =
	    poseEst.getPose(m_intrinsics, m_extrinsics, m_depth_img, m_keyPoints, rect, classId, debugImg);

	double duration = (((double)cv::getTickCount() - startTime) / cv::getTickFrequency() * 1000);  // milliseconds

	constexpr float toleranceMeter = 0.0025;                         // 2.5 mm
	constexpr float toleranceRadians = 2.0 * 0.017444444444444446;  // 0.01744 is approx. one degree
	EXPECT_NEAR(poseRobotCoords.x, m_desiredPose.x, toleranceMeter);
	EXPECT_NEAR(poseRobotCoords.y, m_desiredPose.y, toleranceMeter);
	EXPECT_NEAR(poseRobotCoords.z, m_desiredPose.z, toleranceMeter);
	EXPECT_NEAR(poseRobotCoords.roll, m_desiredPose.roll, toleranceRadians);
	EXPECT_NEAR(poseRobotCoords.pitch, m_desiredPose.pitch, toleranceRadians);
	EXPECT_NEAR(poseRobotCoords.yaw, m_desiredPose.yaw, toleranceRadians);
}

// execute same poseEstimation multiple times and measure time (caching effects)
TEST_F(PoseEstimationTest, multiplePoseEstimations)
{
	VSPose poseCamCoords;
	fs::path networksConfigPath =
	    utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/poseEstimation/networksConfig.json";
	PoseEstimation poseEst(networksConfigPath);
	VSPose poseRobotCoords;
	double duration = 0;

	cv::Mat debugImg = cv::Mat::zeros(m_mask.size(), CV_8UC3);
	for (int i = 0; i < 5; ++i)
	{
		double startTime = (double)cv::getTickCount();
		cv::Rect rect = cv::boundingRect(m_keyPoints);
		int classId = 0;
		poseRobotCoords =
		    poseEst.getPose(m_intrinsics, m_extrinsics, m_depth_img, m_keyPoints, rect, classId, debugImg);
		duration = (((double)cv::getTickCount() - startTime) / cv::getTickFrequency() * 1000);  // milliseconds
	}

	constexpr float toleranceMeter = 0.0025;                         // 2.5 mm
	constexpr float toleranceRadians = 2.0 * 0.017444444444444446;  // 0.01744 is approx. one degree
	EXPECT_NEAR(poseRobotCoords.x, m_desiredPose.x, toleranceMeter);
	EXPECT_NEAR(poseRobotCoords.y, m_desiredPose.y, toleranceMeter);
	EXPECT_NEAR(poseRobotCoords.z, m_desiredPose.z, toleranceMeter);
	EXPECT_NEAR(poseRobotCoords.roll, m_desiredPose.roll, toleranceRadians);
	EXPECT_NEAR(poseRobotCoords.pitch, m_desiredPose.pitch, toleranceRadians);
	EXPECT_NEAR(poseRobotCoords.yaw, m_desiredPose.yaw, toleranceRadians);
	// EXPECT_LT(duration, 10.0);  // milliseconds
}

TEST_F(PoseEstimationTest, longShortEdge)
{
	VSPose poseCamCoords;

	fs::path networksConfigPath =
	    utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/poseEstimation/networksConfigLongShortEdge.json";
	PoseEstimation poseEst(networksConfigPath);
	cv::Mat debugImg =
	    cv::imread(utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/poseEstimation/srcImg.png");
	m_depth_img = cv::imread("/home/cobot/Downloads/00_unitTestPipeline/testfiles/poseEstimation/depthImg.tiff",
	                         cv::IMREAD_ANYDEPTH);

	nlohmann::json jsonFileCamera = utils::loadJsonFile(
	    "/home/cobot/Downloads/00_unitTestPipeline/testfiles/poseEstimation/pickCameraLongShortEdge.json");
	m_intrinsics.fx = jsonFileCamera.at("camera").at(0).at("fx");
	m_intrinsics.fy = jsonFileCamera.at("camera").at(0).at("fy");
	m_intrinsics.cx = jsonFileCamera.at("camera").at(0).at("cx");
	m_intrinsics.cy = jsonFileCamera.at("camera").at(0).at("cy");

	m_extrinsics.at<double>(0, 0) = jsonFileCamera.at("camera").at(0).at("r_11");
	m_extrinsics.at<double>(0, 1) = jsonFileCamera.at("camera").at(0).at("r_12");
	m_extrinsics.at<double>(0, 2) = jsonFileCamera.at("camera").at(0).at("r_13");
	m_extrinsics.at<double>(1, 0) = jsonFileCamera.at("camera").at(0).at("r_21");
	m_extrinsics.at<double>(1, 1) = jsonFileCamera.at("camera").at(0).at("r_22");
	m_extrinsics.at<double>(1, 2) = jsonFileCamera.at("camera").at(0).at("r_23");
	m_extrinsics.at<double>(2, 0) = jsonFileCamera.at("camera").at(0).at("r_31");
	m_extrinsics.at<double>(2, 1) = jsonFileCamera.at("camera").at(0).at("r_32");
	m_extrinsics.at<double>(2, 2) = jsonFileCamera.at("camera").at(0).at("r_33");

	// offsets
	m_extrinsics.at<double>(0, 3) = jsonFileCamera.at("camera").at(0).at("x_offset");
	m_extrinsics.at<double>(1, 3) = jsonFileCamera.at("camera").at(0).at("y_offset");
	m_extrinsics.at<double>(2, 3) = jsonFileCamera.at("camera").at(0).at("z_offset");

	nlohmann::json jsonFileNetworksConfig = utils::loadJsonFile(networksConfigPath);
	float shrinkKeyPointsToCenterVal =
	    jsonFileNetworksConfig.at("appConfigs").at("dlcs").at(0).at("detectionConfigs").at("shrinkKeyPointsToCenterVal").get<float>();
	fs::path keypointsPath =
	    "/home/cobot/Downloads/00_unitTestPipeline/testfiles/poseEstimation/dataLongShortEdge.json";
	nlohmann::json jsonFile = utils::loadJsonFile(keypointsPath);
	cv::Rect rect = cv::Rect(jsonFile.at("rect").at("x"),
	                         jsonFile.at("rect").at("y"),
	                         jsonFile.at("rect").at("width"),
	                         jsonFile.at("rect").at("height"));
	cv::Point2f centerOfRect = {rect.x + rect.width * 0.5f, rect.y + rect.height * 0.5f};

	std::vector<cv::Point2f> keyPointsEdge;
	for (int i = 0; i < jsonFile.at("keyPoints").size(); ++i)
	{
		int x = jsonFile.at("keyPoints").at(i).at(0);
		int y = jsonFile.at("keyPoints").at(i).at(1);
		x = (x - centerOfRect.x) * shrinkKeyPointsToCenterVal + centerOfRect.x;  // avoid wrong depth values at the edge
		y = (y - centerOfRect.y) * shrinkKeyPointsToCenterVal + centerOfRect.y;
		keyPointsEdge.emplace_back(cv::Point2f(x, y));
	}

	double startTime = (double)cv::getTickCount();
	int classId = 0;

	// m_depth_img = cv::Mat::ones(m_depth_img.size(), CV_32FC1) * 0.3;
	VSPose poseRobotCoords =
	    poseEst.getPose(m_intrinsics, m_extrinsics, m_depth_img, keyPointsEdge, rect, classId, debugImg);

	double duration = (((double)cv::getTickCount() - startTime) / cv::getTickFrequency() * 1000);  // milliseconds
	std::cout << "duration = " << duration << std::endl;
	// cv::imshow("debugImg", debugImg);
	// cv::waitKey(0);

	constexpr float toleranceMeter = 0.0025;                         // 2.5 mm
	constexpr float toleranceRadians = 2.0 * 0.017444444444444446;  // 0.01744 is approx. one degree
	EXPECT_NEAR(poseRobotCoords.x, -0.071330763399600983, toleranceMeter);
	EXPECT_NEAR(poseRobotCoords.y, -0.70500791072845459, toleranceMeter);
	EXPECT_NEAR(poseRobotCoords.z, 0.29708263278007507, toleranceMeter);
	EXPECT_NEAR(poseRobotCoords.roll, 0.090186968445777893, toleranceRadians);
	EXPECT_NEAR(poseRobotCoords.pitch, 0.0085757607594132423, toleranceRadians);
	EXPECT_NEAR(poseRobotCoords.yaw, 2.7477540969848633, toleranceRadians);
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
