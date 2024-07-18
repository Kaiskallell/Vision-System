/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "stereoCalibration.h"

#include <opencv2/aruco.hpp>

StereoCalibration::StereoCalibration(const fs::path& networksConfigPath, const fs::path& cameraConfigPath)
{
	nlohmann::json jsonFile = utils::loadJsonFile(networksConfigPath);
	m_calibrationBoardPath = jsonFile.at("stereoCalibration").at("calibrationBoardConfigPath").get<std::string>();
	size_t initWidth = jsonFile.at("stereoCalibration").at("initSize").at("width").get<size_t>();
	size_t initHeight = jsonFile.at("stereoCalibration").at("initSize").at("height").get<size_t>();
	m_initSize = cv::Size(initWidth, initHeight);

	size_t targetWidth = jsonFile.at("stereoCalibration").at("targetSize").at("width").get<size_t>();
	size_t targetHeight = jsonFile.at("stereoCalibration").at("targetSize").at("height").get<size_t>();
	m_targetSize = cv::Size(targetWidth, targetHeight);

	m_uOffset = jsonFile.at("stereoCalibration").at("uOffset").get<size_t>();
	m_vOffset = jsonFile.at("stereoCalibration").at("vOffset").get<size_t>();

	m_imgL = cv::imread(jsonFile.at("stereoCalibration").at("pathToImgOfLeftCam"));
	m_imgR = cv::imread(jsonFile.at("stereoCalibration").at("pathToImgOfRightCam"));

	std::string serialNbrL = jsonFile.at("stereoCalibration").at("serialNbrOfLeftCam").get<std::string>();
	std::string serialNbrR = jsonFile.at("stereoCalibration").at("serialNbrOfRightCam").get<std::string>();

	m_outputFileName = jsonFile.at("stereoCalibration").at("outputFileName").get<std::string>();

	// get intrinsics from cameraConfigPath
	jsonFile = utils::loadJsonFile(cameraConfigPath);
	int camIdxCamCofigL = utils::getCamIdxFromCameraConfig(serialNbrL, cameraConfigPath);
	double fxL = jsonFile.at("camera")[camIdxCamCofigL].at("fx").get<double>();
	double fyL = jsonFile.at("camera")[camIdxCamCofigL].at("fy").get<double>();
	double cxL = jsonFile.at("camera")[camIdxCamCofigL].at("cx").get<double>();
	double cyL = jsonFile.at("camera")[camIdxCamCofigL].at("cy").get<double>();

	double k1L = jsonFile.at("camera")[camIdxCamCofigL].at("distCoeffs")[0].get<double>();
	double k2L = jsonFile.at("camera")[camIdxCamCofigL].at("distCoeffs")[1].get<double>();
	double p1L = jsonFile.at("camera")[camIdxCamCofigL].at("distCoeffs")[2].get<double>();
	double p2L = jsonFile.at("camera")[camIdxCamCofigL].at("distCoeffs")[3].get<double>();
	double k3L = jsonFile.at("camera")[camIdxCamCofigL].at("distCoeffs")[4].get<double>();
	m_intrinsicL = (cv::Mat_<double>(3, 3) << fxL,
	                0.00000000e+00,
	                cxL,
	                0.00000000e+00,
	                fyL,
	                cyL,
	                0.00000000e+00,
	                0.00000000e+00,
	                1.00000000e+00);
	m_distCoeffsL = (cv::Mat_<double>(5, 1) << k1L, k2L, p1L, p2L, k3L);

	int camIdxCamCofigR = utils::getCamIdxFromCameraConfig(serialNbrR, cameraConfigPath);
	double fxR = jsonFile.at("camera")[camIdxCamCofigR].at("fx").get<double>();
	double fyR = jsonFile.at("camera")[camIdxCamCofigR].at("fy").get<double>();
	double cxR = jsonFile.at("camera")[camIdxCamCofigR].at("cx").get<double>();
	double cyR = jsonFile.at("camera")[camIdxCamCofigR].at("cy").get<double>();

	double k1R = jsonFile.at("camera")[camIdxCamCofigR].at("distCoeffs")[0].get<double>();
	double k2R = jsonFile.at("camera")[camIdxCamCofigR].at("distCoeffs")[1].get<double>();
	double p1R = jsonFile.at("camera")[camIdxCamCofigR].at("distCoeffs")[2].get<double>();
	double p2R = jsonFile.at("camera")[camIdxCamCofigR].at("distCoeffs")[3].get<double>();
	double k3R = jsonFile.at("camera")[camIdxCamCofigR].at("distCoeffs")[4].get<double>();
	m_intrinsicR = (cv::Mat_<double>(3, 3) << fxR,
	                0.00000000e+00,
	                cxR,
	                0.00000000e+00,
	                fyR,
	                cyR,
	                0.00000000e+00,
	                0.00000000e+00,
	                1.00000000e+00);
	m_distCoeffsR = (cv::Mat_<double>(5, 1) << k1R, k2R, p1R, p2R, k3R);
}

StereoCalibration::~StereoCalibration() {}

// Detect calibration board and return the detected marker IDs and flattened
// corner points
void StereoCalibration::detectCalibrationBoard(const cv::Mat& img,
                                               std::vector<int>& ids,
                                               std::vector<std::vector<cv::Point2f>>& corners)
{
	// Create an ArUco dictionary and set the detector parameters
	cv::Ptr<cv::aruco::Dictionary> arucoDict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
	cv::Ptr<cv::aruco::DetectorParameters> arucoParams = cv::aruco::DetectorParameters::create();
	arucoParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
	arucoParams->cornerRefinementMaxIterations = 100;
	arucoParams->cornerRefinementMinAccuracy = 0.0001;

	// Convert the image to grayscale
	cv::Mat img_gray;
	cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

	// Detect ArUco markers in the image
	cv::aruco::detectMarkers(img_gray, arucoDict, corners, ids, arucoParams);
}

// Generate 3D object points for the calibration board
std::vector<std::vector<cv::Point3f>> StereoCalibration::generateObjectPoints(const std::vector<int>& ids,
                                                                              const std::string& calibrationboardPath)
{
	nlohmann::json calib_data;
	// Read the calibration board data from a JSON file
	try
	{
		std::ifstream json_calib(calibrationboardPath);
		json_calib >> calib_data;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << '\n';
		std::cerr << "no  file in calibrationboardPath = " << calibrationboardPath << std::endl;
	}

	// Extract the marker length from the calibration data
	float marker_length = calib_data["markerlength"];
	std::vector<std::vector<cv::Point3f>> objectPoints;
	objectPoints.emplace_back(std::vector<cv::Point3f>());

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

		objectPoints.at(0).emplace_back(top_left);
		objectPoints.at(0).emplace_back(top_right);
		objectPoints.at(0).emplace_back(bottom_right);
		objectPoints.at(0).emplace_back(bottom_left);
	}

	return objectPoints;
}

void StereoCalibration::calibrateStereo3D()
{
	std::vector<int> idsL;
	std::vector<std::vector<cv::Point2f>> cornersL;
	detectCalibrationBoard(m_imgL, idsL, cornersL);
	std::vector<int> idsR;
	std::vector<std::vector<cv::Point2f>> cornersR;
	detectCalibrationBoard(m_imgR, idsR, cornersR);

	std::vector<int> idsCommon;
	std::vector<std::vector<cv::Point2f>> cornersLCommon;
	std::vector<std::vector<cv::Point2f>> cornersRCommon;
	for (int i = 0; i < idsL.size(); ++i)
	{
		for (int j = 0; j < idsR.size(); ++j)
		{
			if (idsL.at(i) == idsR.at(j))
			{
				idsCommon.emplace_back(idsL.at(i));
				cornersLCommon.emplace_back(cornersL.at(i));
				cornersRCommon.emplace_back(cornersR.at(j));
			}
		}
	}

	std::vector<std::vector<cv::Point2f>> cornersLFlat;
	cornersLFlat.emplace_back(std::vector<cv::Point2f>());
	std::vector<std::vector<cv::Point2f>> cornersRFlat;
	cornersRFlat.emplace_back(std::vector<cv::Point2f>());
	for (const auto& point : cornersLCommon)
	{
		cornersLFlat.at(0).insert(cornersLFlat.at(0).end(), point.begin(), point.end());
	}
	for (const auto& point : cornersRCommon)
	{
		cornersRFlat.at(0).insert(cornersRFlat.at(0).end(), point.begin(), point.end());
	}
	
	cornersL = cornersLFlat;
	cornersR = cornersRFlat;

	cv::Mat intrinsicsLeft = m_intrinsicL.clone();
	cv::Mat intrinsicsRight = m_intrinsicR.clone();

	nlohmann::json boardConfigJson = nlohmann::json(m_calibrationBoardPath);
	std::vector<std::vector<cv::Point3f>> objPoints = generateObjectPoints(idsCommon, m_calibrationBoardPath);

	cv::Mat grayImgL;
	cv::cvtColor(m_imgL, grayImgL, cv::COLOR_BGR2GRAY);

	cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
	                          30,        // max number of iterations
	                          0.00001);  // min accuracy

	cv::Mat rot;
	cv::Mat trans;
	cv::Mat essentialMat;
	cv::Mat fundamentalMat;
	int flags = cv::CALIB_FIX_INTRINSIC;
	double errStereo = cv::stereoCalibrate(objPoints,
	                                       cornersL,
	                                       cornersR,
	                                       intrinsicsLeft,
	                                       m_distCoeffsL,
	                                       intrinsicsRight,
	                                       m_distCoeffsR,
	                                       grayImgL.size(),
	                                       rot,
	                                       trans,
	                                       essentialMat,
	                                       fundamentalMat,
	                                       flags,
	                                       criteria);

	intrinsicsLeft.at<double>(0, 2) = intrinsicsLeft.at<double>(0, 2) - m_uOffset;
	intrinsicsLeft.at<double>(1, 2) = intrinsicsLeft.at<double>(1, 2) - m_vOffset;

	intrinsicsRight.at<double>(0, 2) = intrinsicsRight.at<double>(0, 2) - m_uOffset;
	intrinsicsRight.at<double>(1, 2) = intrinsicsRight.at<double>(1, 2) - m_vOffset;

	double scalingFactor = static_cast<double>(m_targetSize.width) / static_cast<double>(m_initSize.width);
	intrinsicsRight = intrinsicsRight * scalingFactor;
	intrinsicsLeft = intrinsicsLeft * scalingFactor;

	// stereo rectification
	double rectifyScale = 0.0;
	cv::Mat rectL;
	cv::Mat rectR;
	cv::Mat projMatL;
	cv::Mat projMatR;
	cv::Mat Q;
	cv::Rect roiL;
	cv::Rect roiR;
	constexpr double kalpha = -1;
	cv::stereoRectify(intrinsicsLeft,
	                  m_distCoeffsL,
	                  intrinsicsRight,
	                  m_distCoeffsR,
	                  m_targetSize,
	                  rot,
	                  trans,
	                  rectL,
	                  rectR,
	                  projMatL,
	                  projMatR,
	                  Q,
	                  cv::CALIB_ZERO_DISPARITY,
	                  rectifyScale,
	                  cv::Size(0, 0));

	// rectL needs to have the same dimension like Q otherwise matrix multiplication does not work
	// but is needed in stereoDepth class postProcessing
	cv::Mat rectL4x4 = (cv::Mat_<double>(4, 4) << rectL.at<double>(cv::Point(0, 0)),
	                    rectL.at<double>(cv::Point(1, 0)),
	                    rectL.at<double>(cv::Point(2, 0)),
	                    0,
	                    rectL.at<double>(cv::Point(0, 1)),
	                    rectL.at<double>(cv::Point(1, 1)),
	                    rectL.at<double>(cv::Point(2, 1)),
	                    0,
	                    rectL.at<double>(cv::Point(0, 2)),
	                    rectL.at<double>(cv::Point(1, 2)),
	                    rectL.at<double>(cv::Point(2, 2)),
	                    0,
	                    0,
	                    0,
	                    0,
	                    1);

	try
	{
		fs::path stereoConfigFilePath = utils::getProjectRootDir() / "config" / m_outputFileName;
		cv::FileStorage fstorage(stereoConfigFilePath, cv::FileStorage::WRITE);
		fstorage.write("CameraMatrixL", m_intrinsicL);
		fstorage.write("CameraMatrixR", m_intrinsicR);
		fstorage.write("distL", m_distCoeffsL);
		fstorage.write("distR", m_distCoeffsR);
		fstorage.write("rot", rot);
		fstorage.write("trans", trans);
		fstorage.write("Q", Q);
		fstorage.write("rectL", rectL4x4);
		fstorage.write("rectR", rectR);
		fstorage.write("projMatrixL", projMatL);
		fstorage.write("projMatrixR", projMatR);

		cv::Mat stereoMapL_x;
		cv::Mat stereoMapL_y;
		cv::Mat stereoMapR_x;
		cv::Mat stereoMapR_y;
		cv::Mat stereoMapL_inv_x;
		cv::Mat stereoMapL_inv_y;
		cv::initUndistortRectifyMap(
		    intrinsicsLeft, m_distCoeffsL, rectL, projMatL, m_targetSize, CV_16SC2, stereoMapL_x, stereoMapL_y);
		cv::initUndistortRectifyMap(
		    intrinsicsRight, m_distCoeffsR, rectR, projMatR, m_targetSize, CV_16SC2, stereoMapR_x, stereoMapR_y);
		cv::initInverseRectificationMap(
		    intrinsicsLeft, m_distCoeffsL, rectL, projMatL, m_targetSize, CV_16SC2, stereoMapL_inv_x, stereoMapL_inv_y);
		fstorage.write("stereoMapL_x", stereoMapL_x);
		fstorage.write("stereoMapL_y", stereoMapL_y);
		fstorage.write("stereoMapR_x", stereoMapR_x);
		fstorage.write("stereoMapR_y", stereoMapR_y);
		fstorage.write("stereoMapL_inv_x", stereoMapL_inv_x);
		fstorage.write("stereoMapL_inv_y", stereoMapL_inv_y);
		fstorage.release();
	}
	catch (const std::exception& e)
	{
		throw std::runtime_error(std::string("stereoCalibration saving xml: ") + e.what());
	}
}
