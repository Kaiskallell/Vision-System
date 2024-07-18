/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "calibration.h"

#include <boost/process.hpp>

#include <unistd.h>

#include <fstream>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "cameraFactory/cameraFactory.hpp"
#include "handEyeAlgo.h"
#include "json.hpp"
#include "projectPaths.h"
#include "stereoCalibration.h"

Calibration::Calibration(const fs::path& cameraConfigPath,
                         const fs::path& visionSystemConfigPath,
                         const fs::path& networksConfigPath)
    : m_networksConfigPath(networksConfigPath), m_cameraConfigPath(cameraConfigPath)

{
	m_dbFacade = std::make_unique<db::DbFacade>(visionSystemConfigPath);
	// read the poses from file which should be used for capturing imgs for calibration
	nlohmann::json jsonFile = utils::loadJsonFile(m_networksConfigPath);
	
	m_calcIntrinsicsEnable = boost::algorithm::contains(m_networksConfigPath.string(), "singleCameraCalibration");
	m_stereoCalibEnabled = boost::algorithm::contains(m_networksConfigPath.string(), "stereoCalibration");
	m_calcExtrinsicsEnable = boost::algorithm::contains(m_networksConfigPath.string(), "handEyeCalibration");

	if(!m_stereoCalibEnabled)
	{
		m_ImagePath = jsonFile.at("calibrationConfigs").at("ImagePath").get<fs::path>();
	}

	if(m_calcExtrinsicsEnable)
	{
		m_desiredPose.x = jsonFile.at("calibrationConfigs").at("poses")[0].at("calibPoses")[0].at("coord_x").get<float>();
		m_desiredPose.y = jsonFile.at("calibrationConfigs").at("poses")[0].at("calibPoses")[0].at("coord_y").get<float>();
		m_desiredPose.z = jsonFile.at("calibrationConfigs").at("poses")[0].at("calibPoses")[0].at("coord_z").get<float>();
		m_desiredPose.roll = jsonFile.at("calibrationConfigs").at("poses")[0].at("calibPoses")[0].at("roll").get<float>();
		m_desiredPose.pitch = jsonFile.at("calibrationConfigs").at("poses")[0].at("calibPoses")[0].at("pitch").get<float>();
		m_desiredPose.yaw = jsonFile.at("calibrationConfigs").at("poses")[0].at("calibPoses")[0].at("yaw").get<float>();
		HandEyeAlgo handEye;
		handEye.cvtLandRUnitToSiUnit(m_desiredPose);  // L&R use ° and mm but we need SI-Units
	}
}

void Calibration::writeIntrinsicsToJsonFile(const size_t camConfigIdx,
                                            const cv::Mat& intrinsics,
                                            const cv::Mat& distCoeffs)
{
	fs::path cameraConfigPath = utils::getProjectRootDir() / "config/pickCamera.json";
	nlohmann::json jsonFile = utils::loadJsonFile(cameraConfigPath);
	jsonFile.at("camera").at(camConfigIdx).at("fx") = intrinsics.at<double>(0, 0);
	jsonFile.at("camera").at(camConfigIdx).at("fy") = intrinsics.at<double>(1, 1);
	jsonFile.at("camera").at(camConfigIdx).at("cx") = intrinsics.at<double>(0, 2);
	jsonFile.at("camera").at(camConfigIdx).at("cy") = intrinsics.at<double>(1, 2);
	jsonFile.at("camera").at(camConfigIdx).at("distCoeffs") = {distCoeffs.at<double>(0, 0),
	                                                           distCoeffs.at<double>(0, 1),
	                                                           distCoeffs.at<double>(0, 2),
	                                                           distCoeffs.at<double>(0, 3),
	                                                           distCoeffs.at<double>(0, 4)};

	dumpJsonToDisk(jsonFile, cameraConfigPath);
}

void Calibration::readIntrinsics(const int camConfigIdx, cv::Mat& intrinsics, cv::Mat& distCoeffs)
{
	intrinsics = cv::Mat::eye(cv::Size(3, 3), CV_64FC1);
	distCoeffs = cv::Mat::zeros(cv::Size(5, 1), CV_64FC1);
	fs::path cameraConfigPath = utils::getProjectRootDir() / "config/pickCamera.json";
	nlohmann::json jsonFile = utils::loadJsonFile(cameraConfigPath);
	intrinsics.at<double>(0, 0) = jsonFile.at("camera").at(camConfigIdx).at("fx").get<double>();
	intrinsics.at<double>(1, 1) = jsonFile.at("camera").at(camConfigIdx).at("fy").get<double>();
	intrinsics.at<double>(0, 2) = jsonFile.at("camera").at(camConfigIdx).at("cx").get<double>();
	intrinsics.at<double>(1, 2) = jsonFile.at("camera").at(camConfigIdx).at("cy").get<double>();
	distCoeffs.at<double>(0, 0) = jsonFile.at("camera").at(camConfigIdx).at("distCoeffs").at(0).get<double>();
	distCoeffs.at<double>(0, 1) = jsonFile.at("camera").at(camConfigIdx).at("distCoeffs").at(1).get<double>();
	distCoeffs.at<double>(0, 2) = jsonFile.at("camera").at(camConfigIdx).at("distCoeffs").at(2).get<double>();
	distCoeffs.at<double>(0, 3) = jsonFile.at("camera").at(camConfigIdx).at("distCoeffs").at(3).get<double>();
	distCoeffs.at<double>(0, 4) = jsonFile.at("camera").at(camConfigIdx).at("distCoeffs").at(4).get<double>();
}

void Calibration::writeExtrinsicsToJsonFile(const size_t camConfigIdx, const cv::Mat& extrinsics)
{
	fs::path cameraConfigPath = utils::getProjectRootDir() / "config/pickCamera.json";
	nlohmann::json jsonFile = utils::loadJsonFile(cameraConfigPath);
	jsonFile.at("camera").at(camConfigIdx).at("x_offset") = extrinsics.at<double>(0, 3);
	jsonFile.at("camera").at(camConfigIdx).at("y_offset") = extrinsics.at<double>(1, 3);
	jsonFile.at("camera").at(camConfigIdx).at("z_offset") = extrinsics.at<double>(2, 3);
	jsonFile.at("camera").at(camConfigIdx).at("r_11") = extrinsics.at<double>(0, 0);
	jsonFile.at("camera").at(camConfigIdx).at("r_12") = extrinsics.at<double>(0, 1);
	jsonFile.at("camera").at(camConfigIdx).at("r_13") = extrinsics.at<double>(0, 2);
	jsonFile.at("camera").at(camConfigIdx).at("r_21") = extrinsics.at<double>(1, 0);
	jsonFile.at("camera").at(camConfigIdx).at("r_22") = extrinsics.at<double>(1, 1);
	jsonFile.at("camera").at(camConfigIdx).at("r_23") = extrinsics.at<double>(1, 2);
	jsonFile.at("camera").at(camConfigIdx).at("r_31") = extrinsics.at<double>(2, 0);
	jsonFile.at("camera").at(camConfigIdx).at("r_32") = extrinsics.at<double>(2, 1);
	jsonFile.at("camera").at(camConfigIdx).at("r_33") = extrinsics.at<double>(2, 2);

	dumpJsonToDisk(jsonFile, cameraConfigPath);
}

void Calibration::dumpJsonToDisk(const nlohmann::json jsonFile, const fs::path path2JsonFile)
{
	std::ofstream ofs(path2JsonFile.string());
	if (!ofs.is_open())
	{
		std::string errMsg = "Could not write parameters back to json file: " + path2JsonFile.string();
		throw std::runtime_error(errMsg);
	}
	int indent = 4;
	ofs << jsonFile.dump(indent) << std::endl;  // makes pretty layout
	ofs.close();
}

int Calibration::appendCamToCamConfFile(const int netConfIdx,
                                        const fs::path networkConfigPath,
                                        const fs::path cameraConfigPath)
{
	nlohmann::json jsonFile = utils::loadJsonFile(networkConfigPath);
	std::string serialNo = jsonFile.at("cameraConfigs").at("camera").at(netConfIdx).at("serialNumbers").at(0);

	nlohmann::json camParamObj = {{"serialNumber", serialNo},
	                              {"fx", -1.0},
	                              {"fy", -1.0},
	                              {"cx", -1.0},
	                              {"cy", -1.0},
	                              {"name", "Automatic generated name from hand-eye"},
	                              {
	                                  "distCoeffs",
	                                  {
	                                      -1.0,
	                                      -1.0,
	                                      -1.0,
	                                      -1.0,
	                                      -1.0
	                                  }  //
	                              },
	                              {"r_11", -1.0},
	                              {"r_12", -1.0},
	                              {"r_13", -1.0},
	                              {"r_21", -1.0},
	                              {"r_22", -1.0},
	                              {"r_23", -1.0},
	                              {"r_31", -1.0},
	                              {"r_32", -1.0},
	                              {"r_33", -1.0},
	                              {"x_offset", -1.0},
	                              {"y_offset", -1.0},
	                              {"z_offset", -1.0}

	};

	jsonFile = utils::loadJsonFile(cameraConfigPath);
	jsonFile.at("camera").push_back(camParamObj);
	int camConfIdx = jsonFile.at("camera").size()-1;

	dumpJsonToDisk(jsonFile, cameraConfigPath);
	return camConfIdx;
}

void Calibration::start()
{
	if(m_stereoCalibEnabled)
	{
		m_kLogger->debug("start stereo calibration");
		StereoCalibration stereoCalib(m_networksConfigPath, m_cameraConfigPath);
		stereoCalib.calibrateStereo3D();
		return;
	}
	else
	{
		const fs::path cameraConfigPath = utils::getProjectRootDir() / "config/pickCamera.json";
		int camConfIdx = -1;
		
		try
		{
			camConfIdx = utils::cvtNetConfIdx2CamConfIdx(0, m_networksConfigPath, cameraConfigPath);
		}
		catch (const std::exception& e)
		{
			m_kLogger->error(e.what());
			m_kLogger->debug("Appending a new camera in {} ...", cameraConfigPath);
			camConfIdx = appendCamToCamConfFile(0, m_networksConfigPath, cameraConfigPath);
			if(m_calcExtrinsicsEnable)
			{
				m_kLogger->error("can not calculate extrinsics, because intrinsics parameter were not calculated");
				return;
			}
		}

		cv::Mat intrinsics;
		cv::Mat distCoeffs;
		cv::Mat extrinsics;
		HandEyeAlgo handeyealgo3dBoard;
		if (m_calcIntrinsicsEnable)
		{
			m_kLogger->debug("calc Intrinsics ...");
			handeyealgo3dBoard.calcIntrinsics(m_ImagePath, intrinsics, distCoeffs);
			writeIntrinsicsToJsonFile(camConfIdx, intrinsics, distCoeffs);
			return;
		}
		else if (m_calcExtrinsicsEnable)
		{
			readIntrinsics(camConfIdx, intrinsics, distCoeffs);

			m_kLogger->debug("calc Extrinsics ...");
			cv::Mat measuredPoseImg = cv::imread(m_ImagePath);
			handeyealgo3dBoard.calcExtrinsics(intrinsics,
				                              distCoeffs,
			    	                          measuredPoseImg,
			        	                      m_desiredPose,
			            	                  extrinsics);  // does only need one measured pose
			writeExtrinsicsToJsonFile(camConfIdx, extrinsics);
			return;
		}
	}
}
