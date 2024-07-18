/**
 * @copyright Copyright (c) 2020 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * @brief camera interface vision system
 */

#ifndef VS_CAMERA_INTERFACE_HPP
#define VS_CAMERA_INTERFACE_HPP

#include <atomic>
#include <experimental/filesystem>
#include <iostream>

#include "vs.h"
#include "vs_image.hpp"

// Camera Matrix
class VsCameraIntrinsics
{
  public:
	float fx = 0;
	float fy = 0;
	float cx = 0;
	float cy = 0;

	float k1 = 0;
	float k2 = 0;
	float p1 = 0;
	float p2 = 0;
	float k3 = 0;
};

enum class TriggerMode
{
	TriggerOff = 0,
	TriggerSoftware = 1,
	TriggerHardware = 2
};

struct CropConfigs
{
	size_t cropX = 0;
	size_t cropY = 0;
	size_t cropWidth = 0;
	size_t cropHeight = 0;
};

class VsCameraInterface
{
  protected:
	VsFrame m_frame;
	VsCameraIntrinsics m_cameraMatrix;
	cv::Mat_<double> m_extrinsics;
	size_t m_imgWidth = 0;
	size_t m_imgHeight = 0;
	std::string m_type = "";
	std::string m_name = "";
	std::string m_serialNumber = "";

  public:
    bool m_configured = false;
	bool m_colorDesired = true;

	VsCameraInterface() : m_frame(VsFrame(cv::Size(1280, 720))) {}
	virtual ~VsCameraInterface() = default;

	std::string getType() const { return m_type; }
	void setType(std::string& type) { m_type = type; };

	std::string getName() const { return m_name; }
	void setName(std::string& name) { m_name = name; };

	std::string getSerialNumber() const { return m_serialNumber; }
	void setSerialNumber(std::string& serialNumber) { m_serialNumber = serialNumber; };

	virtual VsFrame vsGetFrame() = 0;
	virtual int close() = 0;

	virtual bool init(TriggerMode triggerMode) = 0;

	void vsClearFrame()
	{
		this->m_frame.colorImage.release();
		this->m_frame.depthImage.release();
	}

	virtual VsCameraIntrinsics vsGetCameraMatrix() = 0;
	virtual cv::Mat_<double> getExtrinsics() = 0;
	virtual size_t getImgWidth() = 0;
	virtual size_t getImgHeight() = 0;
};
#endif
