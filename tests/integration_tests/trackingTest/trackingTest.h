/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 */
#ifndef TRACKING_TEST_H
#define TRACKING_TEST_H

#include "dbFacade/dbFacade.h"
#include "gtest/gtest.h"
#include "logging/logging.h"
#include "objTracking.h"

class TrackingTest : public ::testing::Test, public ObjTracking
{
  public:
	TrackingTest();
	~TrackingTest();
	void run(const std::shared_ptr<InterfaceDetTrk> interfaceDetTrk) override;

  protected:
	bool getNewDataFromDetectionThread(const std::shared_ptr<InterfaceDetTrk> interfaceDetTrk,
	                                   cv::Mat& img,
	                                   std::vector<VSPose>& poses,
	                                   std::vector<int>& classIds,
	                                   uint64_t& frameNumber,
	                                   uint32_t& rotEncoderValue,
	                                   uint32_t& areaNumber,
	                                   TimeStamp& timeStamp,
	                                   bool& frameUpdated);

	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("TrackingTest");
	std::shared_ptr<db::DbFacade> m_dbFacade;
	size_t m_frameNumber = 0;
	cv::FileStorage m_fstorage;
	size_t m_dataCounter = 0;
	size_t m_testIterations = 0;
};

#endif
