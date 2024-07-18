#include <opencv2/aruco/charuco.hpp>

#include "cameraFactory/cameraFactory.hpp"
#include "mathHelperFuncs.h"
#include "miscellaneous.h"
#include "projectPaths.h"
#include "vs_poseObject.h"

void loadConfigs(const fs::path cameraConfigPath,
                 const int camIdx,
                 cv::Mat_<double> cameraMatrix,
                 cv::Mat_<double> distCoeffs)
{
	nlohmann::json jsonFile = utils::loadJsonFile(cameraConfigPath);
	// assign default values
	double fx = 1872.30887281643;
	double fy = 1873.0888862067231;
	double cx = 757.802495329467;
	double cy = 571.4438174442242;

	double k1 = -0.05519215413463856;
	double k2 = -0.8125370038489476;
	double k3 = 0.0009469701579929483;
	double k4 = 0.0005806616774321835;
	double k5 = 4.6242393271253395;
	try
	{
		fx = jsonFile.at("camera")[camIdx].at("fx").get<double>();
		fy = jsonFile.at("camera")[camIdx].at("fy").get<double>();
		cx = jsonFile.at("camera")[camIdx].at("cx").get<double>();
		cy = jsonFile.at("camera")[camIdx].at("cy").get<double>();

		k1 = jsonFile.at("camera")[camIdx].at("distCoeffs")[0].get<double>();
		k2 = jsonFile.at("camera")[camIdx].at("distCoeffs")[1].get<double>();
		k3 = jsonFile.at("camera")[camIdx].at("distCoeffs")[2].get<double>();
		k4 = jsonFile.at("camera")[camIdx].at("distCoeffs")[3].get<double>();
		k5 = jsonFile.at("camera")[camIdx].at("distCoeffs")[4].get<double>();
	}
	catch (const std::exception& e)
	{
		throw::std::runtime_error("Could not read parameters from file: " + cameraConfigPath.string());
	}
	cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
	distCoeffs = (cv::Mat_<double>(1, 5) << k1, k2, k3, k4, k5);
}

int main(int argc, char* argv[])
{
	bool maxResolution = true;
	int camIdx = 0;
	const fs::path& cameraConfigPath = "";
	const fs::path& networksConfigPath = "";
	std::shared_ptr<VsCameraInterface> camera =
	    cameraFactory(camIdx, cameraConfigPath, networksConfigPath, TriggerMode::TriggerOff, maxResolution);

	VsFrame frame = camera->vsGetFrame();
	cv::Mat img = frame.colorImage;
	cv::Mat gray;

	if (img.channels() != 1)
	{
		cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
	}
	else
	{
		gray = img;
	}

	// pease make sure that you are using the correct board dimensions/parameter
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
	std::vector<int> markerIds;
	std::vector<std::vector<cv::Point2f>> markerCorners;
	cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds);
	cv::aruco::drawDetectedMarkers(img, markerCorners, markerIds);

	std::vector<cv::Point2f> charucoCorners;
	std::vector<int> charucoIds;
	cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
	int nInterpCorners =
	    cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, gray, board, charucoCorners, charucoIds);

	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	loadConfigs(cameraConfigPath, camIdx, cameraMatrix, distCoeffs);
	cv::Vec3d rvec;  // these are the output of the pose estimation of the board (rotation vector is pointing in
	                 // the rotation axis, the magnitude is the rotation angle in radians)
	cv::Vec3d tvec;  // translation vector
	bool valid =
	    cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);

	cv::Mat rotationMatBoard;
	cv::Rodrigues(rvec, rotationMatBoard);
	VSPose boardTranslCamCoords;
	boardTranslCamCoords.x = tvec[0];
	boardTranslCamCoords.y = tvec[1];
	boardTranslCamCoords.z = tvec[2];

	// draw the poses
	cv::Mat outputImage = img.clone();
	cv::drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvec, tvec, 0.1f);

	std::cout << "rvec = " << rvec << std::endl;
	std::cout << "tvec = " << tvec << std::endl;

	cv::Mat cam2robot;                             // rotation matrix camera -> robot
	cv::Rect rotPartofMat = cv::Rect(0, 0, 3, 3);  // needed to crop the rotation part (3x3) from extrinsic Matrix (3x4)
	cv::Mat extrinsics = camera->getExtrinsics();
	extrinsics(rotPartofMat).convertTo(cam2robot, CV_32FC1);

	std::cout << rotationMatBoard.type() << std::endl;
	std::cout << cam2robot.type() << std::endl;
	rotationMatBoard.convertTo(rotationMatBoard, CV_32FC1);

	cv::Mat_<double> product2RobotTransformation;  // rotation matrix product -> robot
	product2RobotTransformation = cam2robot * rotationMatBoard;

	VSPose boardTranslRobot = math::convertCam2RobotCoords(boardTranslCamCoords, extrinsics);
	cv::Vec3f vEulerAngles = math::rotationMatrixToEulerAngles(product2RobotTransformation, "ZYX");

	std::cout << "extrinsics = " << extrinsics << std::endl;
	std::cout << "intrinsics = " << cameraMatrix << std::endl;

	std::cout << "boardTranslRobot.x = " << boardTranslRobot.x << std::endl;
	std::cout << "boardTranslRobot.y = " << boardTranslRobot.y << std::endl;
	std::cout << "boardTranslRobot.z = " << boardTranslRobot.z << std::endl;
	std::cout << "Euler Angles = " << vEulerAngles << std::endl;

	std::cout << "boardTranslRobot.x = " << boardTranslRobot.x * 1000.0 << std::endl;
	std::cout << "boardTranslRobot.y = " << boardTranslRobot.y * 1000.0 << std::endl;
	std::cout << "boardTranslRobot.z = " << boardTranslRobot.z * 1000.0 << std::endl;
	std::cout << "Euler Angles = " << vEulerAngles / CV_PI * 180.0 << std::endl;

	// draw detected markers
	cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
	misc::showImage("asd", outputImage);
	cv::waitKey(0);

	return 0;
}
