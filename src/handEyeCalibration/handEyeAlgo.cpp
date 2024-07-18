/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "handEyeAlgo.h"

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

void HandEyeAlgo::cvtLandRUnitToSiUnit(VSPose& pose)
{
	pose.x = pose.x / 1000.0;                 // mm to meter
	pose.y = pose.y / 1000.0;                 // mm to meter
	pose.z = pose.z / 1000.0;                 // mm to meter
	pose.yaw = pose.yaw / 180.0 * CV_PI;      // ° to rad
	pose.pitch = pose.pitch / 180.0 * CV_PI;  // ° to rad
	pose.roll = pose.roll / 180.0 * CV_PI;    // ° to rad
}

HandEyeAlgo::HandEyeAlgo() {}

HandEyeAlgo::~HandEyeAlgo() {}

// Detect calibration board and return the detected marker IDs and flattened corner points
void HandEyeAlgo::detectCalibrationBoard(const cv::Mat& img,
                                         std::vector<int>& ids,
                                         std::vector<cv::Point2f>& flatCorners)
{
	// Create an ArUco dictionary and set the detector parameters
	cv::Ptr<cv::aruco::Dictionary> arucoDict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
	cv::Ptr<cv::aruco::DetectorParameters> arucoParams = cv::aruco::DetectorParameters::create();
	arucoParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

	// Convert the image to grayscale
	cv::Mat img_gray;
	cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

	// Detect ArUco markers in the image
	std::vector<std::vector<cv::Point2f>> corners;
	cv::aruco::detectMarkers(img_gray, arucoDict, corners, ids, arucoParams);

	if (ids.empty())
	{
		m_kLogger->warn("ids are empty");
	}
	if (corners.empty())
	{
		m_kLogger->warn("corners are empty");
	}

	// Flatten the corners vector
	for (const auto& markerCorners : corners)
	{
		flatCorners.insert(flatCorners.end(), markerCorners.begin(), markerCorners.end());
	}
}

// Generate 3D object points for the calibration board
std::vector<cv::Point3f> HandEyeAlgo::generateObjectPoints(const std::vector<int>& ids,
                                                           const std::string& calibrationboard_path)
{
	nlohmann::json calib_data;
	// Read the calibration board data from a JSON file
	try
	{
		std::ifstream json_calib(calibrationboard_path);
		json_calib >> calib_data;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << '\n';
		std::cerr << "no  file in calibrationboard_path = " << calibrationboard_path << std::endl;
	}

	// Extract the marker length from the calibration data
	float marker_length = calib_data["markerlength"];
	std::vector<cv::Point3f> flatObjectPoints;

	// Generate object points for each marker ID
	for (int id : ids)
	{
		const std::vector<float>& top_left_coordinate = calib_data["id" + std::to_string(id)+ "_c"];
		const std::vector<float>& top_right_coordinate = calib_data["id"+ std::to_string(id)+ "_d"];
		const std::vector<float>& bottom_left_coordinate = calib_data["id"+ std::to_string(id)+ "_a"];
		const std::vector<float>& bottom_right_coordinate = calib_data["id"+ std::to_string(id)+ "_b"];
		
		cv::Point3f top_left(top_left_coordinate[0], top_left_coordinate[1], top_left_coordinate[2]);
		cv::Point3f top_right(top_right_coordinate[0], top_right_coordinate[1], top_right_coordinate[2]);
		cv::Point3f bottom_left(bottom_left_coordinate[0], bottom_left_coordinate[1], bottom_left_coordinate[2]);
		cv::Point3f bottom_right(bottom_right_coordinate[0], bottom_right_coordinate[1], bottom_right_coordinate[2]);

		flatObjectPoints.push_back(top_left);
		flatObjectPoints.push_back(top_right);
		flatObjectPoints.push_back(bottom_right);
		flatObjectPoints.push_back(bottom_left);
	}

	return flatObjectPoints;
}

cv::Mat HandEyeAlgo::eulerAnglesToRotationMatrix(double roll, double pitch, double yaw)
{
	// Create rotation matrices for each axis
	cv::Mat rotX = (cv::Mat_<double>(4, 4) << 1,
	                0,
	                0,
	                0,
	                0,
	                std::cos(roll),
	                -std::sin(roll),
	                0,
	                0,
	                std::sin(roll),
	                std::cos(roll),
	                0,
	                0,
	                0,
	                0,
	                1);
	cv::Mat rotY = (cv::Mat_<double>(4, 4) << std::cos(pitch),
	                0,
	                std::sin(pitch),
	                0,
	                0,
	                1,
	                0,
	                0,
	                -std::sin(pitch),
	                0,
	                std::cos(pitch),
	                0,
	                0,
	                0,
	                0,
	                1);
	cv::Mat rotZ = (cv::Mat_<double>(4, 4) << std::cos(yaw),
	                -std::sin(yaw),
	                0,
	                0,
	                std::sin(yaw),
	                std::cos(yaw),
	                0,
	                0,
	                0,
	                0,
	                1,
	                0,
	                0,
	                0,
	                0,
	                1);

	cv::Mat rot = rotZ * rotY * rotX;

	return rot;
}

void HandEyeAlgo::estimatePoseCalibrationBoard(const std::vector<int>& ids,
                                               const std::vector<cv::Point2f>& corners,
                                               const cv::Mat& cameraMatrix,
                                               const cv::Mat& distCoeffs,
                                               cv::Mat& rvec,
                                               cv::Mat& tvec)
{
	// Generate object points using the provided marker IDs and calibration board path
	std::vector<cv::Point3f> object_points =
	    generateObjectPoints(ids,
	                         utils::getProjectRootDir() / "config/calibrationboard.json");  // insert
	                                                                                        // path

	// Estimate pose using solvePnP
	cv::solvePnP(object_points, corners, cameraMatrix, distCoeffs, rvec, tvec);
}

void HandEyeAlgo::calcIntrinsics(const fs::path& imgFolderPath, cv::Mat& intrinsics, cv::Mat& distCoeffs)
{
	// Code for the 3DCalibration
	std::vector<std::vector<cv::Point2f>> markerImagePoints;
	std::vector<std::vector<cv::Point3f>> markerObjectPoints;

	// the calibration images are saved in a directory in the resources folder
	auto dirIter = fs::directory_iterator(imgFolderPath);
	for (const auto& filePath : dirIter)
	{
		if (filePath.path().extension() != ".png" && filePath.path().extension() != ".bmp"
		    && filePath.path().extension() != ".jpg")
		{
			throw std::runtime_error("FilePath contains a file which is a unsupported img format: "
			                         + filePath.path().string());
		}

		cv::Mat img = cv::imread(filePath.path());
		m_kLogger->debug("filePath.path().string() = {}", filePath.path().string());
		if (img.empty())
		{
			throw std::runtime_error("image is empty");
		}
		std::vector<int> ids;
		std::vector<cv::Point2f> corners;
		detectCalibrationBoard(img, ids, corners);
		markerImagePoints.push_back(corners);

		std::vector<cv::Point3f> flattendObjPts =
		    generateObjectPoints(ids, utils::getProjectRootDir() / "config/calibrationboard.json");
		if (flattendObjPts.empty())
		{
			throw std::runtime_error("Cannot detect markers in img: " + filePath.path().string());
		}
		markerObjectPoints.push_back(flattendObjPts);
	}

	// take the first picture to get information about the img.size()
	std::vector<cv::String> imgNames;
	cv::glob(imgFolderPath, imgNames);
	std::string firstImageName = imgNames[0];
	cv::Mat img = cv::imread(firstImageName);

	// cv::Mat initCameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat initCameraMatrix = cv::initCameraMatrix2D(markerObjectPoints, markerImagePoints, img.size());

	cv::Mat rvecs;
	cv::Mat tvecs;
	// rvecs & tvecs: for e.g. 15 imgs we get 15 revs, size=15 rows, 1 col each Vec3d
	cv::calibrateCamera(markerObjectPoints,
	                    markerImagePoints,
	                    img.size(),
	                    initCameraMatrix,
	                    distCoeffs,
	                    rvecs,
	                    tvecs,
	                    cv::CALIB_USE_INTRINSIC_GUESS);
	intrinsics = initCameraMatrix;
	m_kLogger->debug("Camera Matrix (intrinsics) Hand Eye Calibration: {}", intrinsics);
}

void HandEyeAlgo::calcExtrinsics(const cv::Mat& intrinsics,
                                 const cv::Mat& distCoeffs,
                                 const cv::Mat& measuredPoseImg,
                                 const VSPose& measuredPose,
                                 cv::Mat& extrinsics)
{
	std::vector<int> ids;
	std::vector<cv::Point2f> corners;
	detectCalibrationBoard(measuredPoseImg, ids, corners);

	cv::Mat rvec, tvec;
	estimatePoseCalibrationBoard(ids, corners, intrinsics, distCoeffs, rvec, tvec);
	tvec = tvec / 1000.0;  // convert from [mm] to [m]
	cv::Vec3d tvecs_C = tvec.at<cv::Vec3d>(0);
	cv::Vec3d rvecs_C = rvec.at<cv::Vec3d>(0);

	cv::Mat rotMatrix_C;
	cv::Rodrigues(rvecs_C, rotMatrix_C);  // Convert rotation vector to rotation matrix

	cv::Mat H_B_C = (cv::Mat_<double>(4, 4) << rotMatrix_C.at<double>(0, 0),
	                 rotMatrix_C.at<double>(0, 1),
	                 rotMatrix_C.at<double>(0, 2),
	                 tvecs_C[0],
	                 rotMatrix_C.at<double>(1, 0),
	                 rotMatrix_C.at<double>(1, 1),
	                 rotMatrix_C.at<double>(1, 2),
	                 tvecs_C[1],
	                 rotMatrix_C.at<double>(2, 0),
	                 rotMatrix_C.at<double>(2, 1),
	                 rotMatrix_C.at<double>(2, 2),
	                 tvecs_C[2],
	                 0,
	                 0,
	                 0,
	                 1);

	cv::Mat H_B_C_inv = H_B_C.inv();

	cv::Mat rotMatrix_R = eulerAnglesToRotationMatrix(measuredPose.roll, measuredPose.pitch, measuredPose.yaw);

	cv::Mat rotX_180 = (cv::Mat_<double>(4, 4) << 1,
	                    0,
	                    0,
	                    0,
	                    0,
	                    std::cos(M_PI),
	                    -std::sin(M_PI),
	                    0,
	                    0,
	                    std::sin(M_PI),
	                    std::cos(M_PI),
	                    0,
	                    0,
	                    0,
	                    0,
	                    1);
	cv::Mat H_B_R = rotMatrix_R * rotX_180;

	// add the translation of robot arm to homogeneous matrix
	H_B_R.at<double>(0, 3) = measuredPose.x;
	H_B_R.at<double>(1, 3) = measuredPose.y;
	H_B_R.at<double>(2, 3) = measuredPose.z;

	extrinsics = H_B_R * H_B_C_inv;

	// //calculate hand eye by doing matrix multiplication of homogeneous matrix between camera and robot
	m_kLogger->debug("Transformation Matrix (extrinsics) Hand Eye Calibration: {}", extrinsics);
}
