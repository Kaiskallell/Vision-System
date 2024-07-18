/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved

 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 *
 */

#ifndef INTERFACE_DET_TRK_H
#define INTERFACE_DET_TRK_H

#include "vs_image.hpp"
#include "vs_poseObject.h"

class InterfaceDetTrk
{
  private:
	inline static std::mutex m_muDetTrk;
	TimeStamp m_timestamp;
	std::vector<VSPose> m_dets;
	std::vector<int> m_classIds;
	uint64_t m_frameNumber = 0;
	uint32_t m_rotEncoderValue = 0;
	uint32_t m_areaNumber = 0;
	bool m_frameUpdated = false;
	cv::Mat m_img;

  public:
	void lockInterface() { m_muDetTrk.lock(); }  // lock when writing to the Dataset

	void unlockInterface() { m_muDetTrk.unlock(); }

	void sendDataToTrk(const TimeStamp newTimestamp,
	                   const cv::Mat& img,
	                   const std::vector<VSPose>& newDets,
	                   const std::vector<int>& classIds,
	                   const uint64_t frameNumber,
	                   const uint32_t rotEncoderValue,
	                   const uint32_t areaNumber,
	                   const bool frameUpdated)
	{
		m_dets.clear();
		m_classIds.clear();
		if (m_muDetTrk.try_lock())  // if mutex is not locked, it locks it -> therfore unlock it afterwards
		{
			m_muDetTrk.unlock();
			throw std::runtime_error("ERROR function sendDataToTrk is called while mutex is unlocked");
		}

		if (!img.empty())
		{
			m_img = img.clone();
		}

		m_frameNumber = frameNumber;
		m_rotEncoderValue = rotEncoderValue;
		m_areaNumber = areaNumber;
		m_frameUpdated = frameUpdated;

		m_timestamp = newTimestamp;
		if (newDets.empty() || classIds.empty())
		{
			return;
		}

		for (int i = 0; i < newDets.size(); i++)
		{
			if (!newDets.empty())
			{
				m_dets.emplace_back(newDets.at(i));
			}
			if (!classIds.empty())
			{
				m_classIds.emplace_back(classIds.at(i));
			}
		}
	}

	TimeStamp getNewDatafromDetTimestamp() { return m_timestamp; }

	cv::Mat getImg() { return m_img; }

	std::vector<VSPose> getNewData() { return m_dets; }

	std::vector<int> getClassIds() { return m_classIds; }

	uint32_t getFrameNumber() { return m_frameNumber; }
	uint32_t getRotEncoderVal() { return m_rotEncoderValue; }
	uint32_t getAreaNumber() { return m_areaNumber; }

	bool getFrameUpdated() { return m_frameUpdated; }
};

#endif
