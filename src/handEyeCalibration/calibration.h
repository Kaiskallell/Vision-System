/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <boost/process.hpp>

#include "cameraInterface.hpp"
#include "dbFacade/dbFacade.h"
#include "vs_poseObject.h"
class Calibration
{
  public:
	Calibration(const fs::path& cameraConfigPath,
	            const fs::path& visionSystemConfigPath,
	            const fs::path& formatConfigPath);
	void start();

  private:
	void writeIntrinsicsToJsonFile(const size_t camConfIdx, const cv::Mat& intrinsics, const cv::Mat& distCoeffs);
	void readIntrinsics(const int camConfigIdx, cv::Mat& intrinsics, cv::Mat& distCoeffs);

	void writeExtrinsicsToJsonFile(const size_t camConfIdx, const cv::Mat& extrinsics);

	int appendCamToCamConfFile(const int netConfIdx, const fs::path networkConfigPath, const fs::path cameraConfigPath);
	void dumpJsonToDisk(const nlohmann::json jsonFile, const fs::path path2JsonFile);

	std::vector<std::vector<cv::Point3f>> generateObjectPoints(const std::vector<int>& ids,
	                                                           const std::string& calibrationboard_path);

	std::unique_ptr<db::DbFacade> m_dbFacade;
	VSPose m_desiredPose;
	fs::path m_ImagePath = "";
	fs::path m_networksConfigPath = "";
	fs::path m_cameraConfigPath = "";
	bool m_stereoCalibEnabled = false;
	bool m_calcIntrinsicsEnable = false;
	bool m_calcExtrinsicsEnable = false;

	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("Calibration");
};

#endif
