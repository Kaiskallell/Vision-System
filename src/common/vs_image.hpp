/**
 * @copyright Copyright (c) 2020 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * @brief Camera Image Class container
 *
 */

#ifndef VS_IMAGE_HPP
#define VS_IMAGE_HPP

#include <iostream>
#include <opencv2/opencv.hpp>

struct TimeStamp
{
	uint32_t year = 0;
	uint32_t month = 0;
	uint32_t day = 0;
	uint32_t hour = 0;
	uint32_t minute = 0;
	uint32_t second = 0;
	uint32_t millisecond = 0;

	void increment1ms()
	{
		millisecond += 1;
		if (millisecond == std::numeric_limits<decltype(millisecond)>::max())
		{
			millisecond = 0;
			second += 1;
		}
	}
};

class VsFrame
{
  public:
	VsFrame()
	{
		colorImage = cv::Mat();
		depthImage = cv::Mat();
	}

	VsFrame(const cv::Size& size)
	{
		colorImage = cv::Mat::zeros(size, CV_8UC3);
		depthImage = cv::Mat::zeros(size, CV_32FC1);
		secondBgrImage = cv::Mat::zeros(size, CV_8UC3);
	}
	cv::Mat colorImage;      // BGR image
	cv::Mat depthImage;      // CV32FC1 image
	cv::Mat secondBgrImage;  // in case of stereo (e.g. right camera img)

	TimeStamp timeStamp;

	long long frameCounter = 0;
};

#endif
