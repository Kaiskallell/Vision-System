/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 */
#ifndef DETECT_FROM_TRAY_TEST_H
#define DETECT_FROM_TRAY_TEST_H

#include "detectFromTray.h"
#include "gtest/gtest.h"
#include "logging/logging.h"

class DetectFromTrayTest : public ::testing::Test, public DetectFromTray
{
  public:
	void run() override;

  protected:
	DetectFromTrayTest();
	void readFromYml(std::vector<cv::Mat>& colorImgs,
	                 std::vector<cv::Mat>& depthImgs,
	                 std::vector<std::vector<VSPose>>& poseVecs);
	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("DetectFromTrayTest");
};

#endif