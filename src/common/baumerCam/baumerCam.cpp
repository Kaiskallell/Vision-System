
/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "baumerCam.hpp"

#include <fstream>

#include "neoapi/neoapi.hpp"
#include "projectPaths.h"

class BaumerCam::Impl
{
  public:
	Impl(){};
	~Impl() = default;
	NeoAPI::Cam m_camera = NeoAPI::Cam();
};

BaumerCam::BaumerCam(const fs::path& cameraConfigPath,
                     const fs::path& networksConfigPath,
                     const std::string& serialNumber,
                     bool maxResolution)
    : m_impl(std::make_shared<Impl>())
{
	m_kLogger->info("Loading config files for baumer camera");
	int camIdx = utils::getCamIdxFromCameraConfig(serialNumber, cameraConfigPath);

	// read intrinsic paramters from pickCamera.json
	nlohmann::json jsonFile = utils::loadJsonFile(cameraConfigPath);

	this->m_extrinsics = cv::Mat_<double>(3, 4);
	try
	{
		this->m_cameraMatrix.fx = jsonFile.at("camera")[camIdx].at("fx").get<float>();
		this->m_cameraMatrix.fy = jsonFile.at("camera")[camIdx].at("fy").get<float>();
		this->m_cameraMatrix.cx = jsonFile.at("camera")[camIdx].at("cx").get<float>();
		this->m_cameraMatrix.cy = jsonFile.at("camera")[camIdx].at("cy").get<float>();

		this->m_cameraMatrix.k1 = jsonFile.at("camera")[camIdx].at("distCoeffs")[0].get<double>();
		this->m_cameraMatrix.k2 = jsonFile.at("camera")[camIdx].at("distCoeffs")[1].get<double>();
		this->m_cameraMatrix.p1 = jsonFile.at("camera")[camIdx].at("distCoeffs")[2].get<double>();
		this->m_cameraMatrix.p2 = jsonFile.at("camera")[camIdx].at("distCoeffs")[3].get<double>();
		this->m_cameraMatrix.k3 = jsonFile.at("camera")[camIdx].at("distCoeffs")[4].get<double>();

		this->m_extrinsics.at<double>(0, 0) = jsonFile.at("camera")[camIdx].at("r_11").get<double>();
		this->m_extrinsics.at<double>(0, 1) = jsonFile.at("camera")[camIdx].at("r_12").get<double>();
		this->m_extrinsics.at<double>(0, 2) = jsonFile.at("camera")[camIdx].at("r_13").get<double>();
		this->m_extrinsics.at<double>(1, 0) = jsonFile.at("camera")[camIdx].at("r_21").get<double>();
		this->m_extrinsics.at<double>(1, 1) = jsonFile.at("camera")[camIdx].at("r_22").get<double>();
		this->m_extrinsics.at<double>(1, 2) = jsonFile.at("camera")[camIdx].at("r_23").get<double>();
		this->m_extrinsics.at<double>(2, 0) = jsonFile.at("camera")[camIdx].at("r_31").get<double>();
		this->m_extrinsics.at<double>(2, 1) = jsonFile.at("camera")[camIdx].at("r_32").get<double>();
		this->m_extrinsics.at<double>(2, 2) = jsonFile.at("camera")[camIdx].at("r_33").get<double>();

		// offsets
		this->m_extrinsics.at<double>(0, 3) = jsonFile.at("camera")[camIdx].at("x_offset").get<double>();
		this->m_extrinsics.at<double>(1, 3) = jsonFile.at("camera")[camIdx].at("y_offset").get<double>();
		this->m_extrinsics.at<double>(2, 3) = jsonFile.at("camera")[camIdx].at("z_offset").get<double>();
	}
	catch (const std::exception& e)
	{
		m_kLogger->error("Could not read parameters from file: " + cameraConfigPath.string());
	}
		this->m_serialNumber = serialNumber;
	NeoAPI::NeoString serialNoBaumer(m_serialNumber.c_str());

	fs::path baumerCamConfigPath = utils::getProjectRootDir() / "config/baumerCamConfig.json";
	jsonFile = utils::loadJsonFile(baumerCamConfigPath);

	// assign default values
	bool autoWhiteBalance = false;
	int whiteBalanceOffsetX = 10;
	int whiteBalanceOffsetY = 10;
	int whiteBalanceWidth = 1024;
	int whiteBalanceHeight = 1024;
	bool autoBrightness = false;
	int brightnessOffsetX = 10;
	int brightnessOffsetY = 10;
	int brightnessWidth = 1024;
	int brightnessHeight = 1024;
	m_baumerConfigs.gain = 1.0;
	// read from path
	try
	{
		autoWhiteBalance = jsonFile.at("autoWhiteBalance").get<bool>();
	    whiteBalanceOffsetX = jsonFile.at("whiteBalanceAutoX").get<int>();
	    whiteBalanceOffsetY = jsonFile.at("whiteBalanceAutoY").get<int>();
	    whiteBalanceWidth = jsonFile.at("whiteBalanceAutoWidth").get<int>();
	    whiteBalanceHeight = jsonFile.at("whiteBalanceAutoHeight").get<int>();

	    autoBrightness = jsonFile.at("autoBrightness").get<bool>();
	    brightnessOffsetX = jsonFile.at("brightnessAutoX").get<int>();
	    brightnessOffsetY = jsonFile.at("brightnessAutoY").get<int>();
	    brightnessWidth = jsonFile.at("brightnessAutoWidth").get<int>();
	    brightnessHeight = jsonFile.at("brightnessAutoHeight").get<int>();
	    m_baumerConfigs.gain = jsonFile.at("gain").get<float>();
	}
	catch (const std::exception& e)
	{
		m_kLogger->error("Could not read parameters from file: " + baumerCamConfigPath.string());
	}

	camIdx = utils::getCamIdxFromNetworksConfig(serialNumber, networksConfigPath);
	jsonFile = utils::loadJsonFile(networksConfigPath);

    bool binningEnabled = false;
	bool reverseX = false;
	bool reverseY = false;
    try
	{
		m_cropConfigs.cropX = jsonFile.at("cameraConfigs").at("camera")[camIdx].at("cropX").get<size_t>();
	    m_cropConfigs.cropY = jsonFile.at("cameraConfigs").at("camera")[camIdx].at("cropY").get<size_t>();
	    m_cropConfigs.cropWidth = jsonFile.at("cameraConfigs").at("camera")[camIdx].at("cropWidth").get<size_t>();
	    m_cropConfigs.cropHeight = jsonFile.at("cameraConfigs").at("camera")[camIdx].at("cropHeight").get<size_t>();
	    m_colorDesired = jsonFile.at("cameraConfigs").at("camera")[camIdx].at("color").get<bool>();

	    m_baumerConfigs.exposureTime = jsonFile.at("cameraConfigs").at("camera")[camIdx].at("exposureTime").get<float>();
	    binningEnabled = jsonFile.at("cameraConfigs").at("camera")[camIdx].at("binningEnabled").get<bool>();
	    reverseX = jsonFile.at("cameraConfigs").at("camera")[camIdx].at("reverseX").get<bool>();
	    reverseY = jsonFile.at("cameraConfigs").at("camera")[camIdx].at("reverseY").get<bool>();
	}
	catch (const std::exception& e)
	{
		m_kLogger->error("Could not read parameters from file: " + networksConfigPath.string());
	}
	try
	{
		m_impl->m_camera.Connect(serialNoBaumer);

		if (m_colorDesired)
		{
			m_impl->m_camera.f().PixelFormat.Set(NeoAPI::PixelFormat::BGR8);
		}
		else
		{
			m_impl->m_camera.f().PixelFormat.Set(NeoAPI::PixelFormat::Mono8);
		}

		m_impl->m_camera.SetAdjustFeatureValueMode(false);  // switch off automated value adjustment
		m_impl->m_camera.f().Gain.Set(m_baumerConfigs.gain);

		if (maxResolution)  // for hand eye calibration
		{
			m_impl->m_camera.f().BinningHorizontal.Set(1);
			m_impl->m_camera.f().BinningVertical.Set(1);
			m_impl->m_camera.f().OffsetX.Set(0);  // baumer support said that it should be initalize with 0 first
			m_impl->m_camera.f().OffsetY.Set(0);  // baumer support said that it should be initalize with 0 first
			// constexpr size_t kMaxPixelWidth = 2048;
			// constexpr size_t kMaxPixelHeight = 1536;
			const size_t kMaxPixelWidth = m_impl->m_camera.f().SensorWidth;
			const size_t kMaxPixelHeight = m_impl->m_camera.f().SensorHeight;
			m_impl->m_camera.f().Width.Set(kMaxPixelWidth);    // then witdth
			m_impl->m_camera.f().Height.Set(kMaxPixelHeight);  // then heigth
		}
		else
		{
			if (binningEnabled)
			{
				m_impl->m_camera.f().BinningHorizontal.Set(2);
				m_impl->m_camera.f().BinningVertical.Set(2);
				this->m_cameraMatrix.cx = this->m_cameraMatrix.cx * 0.5;
				this->m_cameraMatrix.cy = this->m_cameraMatrix.cy * 0.5;
				this->m_cameraMatrix.fx = this->m_cameraMatrix.fx * 0.5;
				this->m_cameraMatrix.fy = this->m_cameraMatrix.fy * 0.5;
				this->m_cameraMatrix.k1 = m_cameraMatrix.k1 * 4.0;
				this->m_cameraMatrix.k2 = m_cameraMatrix.k2 * 16.0;
				this->m_cameraMatrix.p1 = m_cameraMatrix.p1 * 4.0;
				this->m_cameraMatrix.p2 = m_cameraMatrix.p2 * 4.0;
				this->m_cameraMatrix.k3 = m_cameraMatrix.k3 * 256.0;
			}
			else
			{
				m_impl->m_camera.f().BinningHorizontal.Set(1);
				m_impl->m_camera.f().BinningVertical.Set(1);
			}
			m_kLogger->debug("correcting intrinsics according to crop of img");
			this->m_cameraMatrix.cx = this->m_cameraMatrix.cx - m_cropConfigs.cropX;
			this->m_cameraMatrix.cy = this->m_cameraMatrix.cy - m_cropConfigs.cropY;
			m_impl->m_camera.f().OffsetX.Set(0);  // baumer support said that it should be initalize with 0 first
			m_impl->m_camera.f().OffsetY.Set(0);  // baumer support said that it should be initalize with 0 first
			m_impl->m_camera.f().Width.Set(m_cropConfigs.cropWidth);    // then witdth
			m_impl->m_camera.f().Height.Set(m_cropConfigs.cropHeight);  // then heigth
			m_impl->m_camera.f().OffsetX.Set(m_cropConfigs.cropX);      // at the end offset again
			m_impl->m_camera.f().OffsetY.Set(m_cropConfigs.cropY);

			m_impl->m_camera.f().ReverseX.Set(reverseX);
			m_impl->m_camera.f().ReverseY.Set(reverseY);
		}

		if (m_colorDesired)
		{
			if (autoWhiteBalance)
			{
				m_impl->m_camera.f().AutoFeatureRegionSelector.Set(NeoAPI::AutoFeatureRegionSelector::BalanceWhiteAuto);
				m_impl->m_camera.f().AutoFeatureRegionMode.Set(NeoAPI::AutoFeatureRegionMode::On);
				m_impl->m_camera.f().AutoFeatureHeight.Set(whiteBalanceHeight);
				m_impl->m_camera.f().AutoFeatureWidth.Set(whiteBalanceWidth);
				m_impl->m_camera.f().AutoFeatureOffsetX.Set(whiteBalanceOffsetX);
				m_impl->m_camera.f().AutoFeatureOffsetY.Set(whiteBalanceOffsetY);
				m_impl->m_camera.f().BalanceWhiteAuto.Set(NeoAPI::BalanceWhiteAuto::Continuous);
			}
			else
			{
				m_impl->m_camera.f().BalanceWhiteAuto.Set(NeoAPI::BalanceWhiteAuto::Off);
				m_impl->m_camera.f().AutoFeatureRegionMode.Set(NeoAPI::AutoFeatureRegionMode::Off);
			}
		}

		if (autoBrightness)
		{
			m_impl->m_camera.f().AutoFeatureRegionSelector.Set(NeoAPI::AutoFeatureRegionSelector::BrightnessAuto);
			m_impl->m_camera.f().AutoFeatureHeight.Set(brightnessHeight);
			m_impl->m_camera.f().AutoFeatureWidth.Set(brightnessWidth);
			m_impl->m_camera.f().AutoFeatureOffsetX.Set(brightnessOffsetX);
			m_impl->m_camera.f().AutoFeatureOffsetY.Set(brightnessOffsetY);
			m_impl->m_camera.f().AutoFeatureRegionMode.SetString("On");
			m_impl->m_camera.f().BrightnessAutoNominalValue.Set(m_baumerConfigs.brightness);
			m_impl->m_camera.f().ExposureAuto.Set(NeoAPI::ExposureAuto::Continuous);
		}
		else
		{
			m_impl->m_camera.f().ExposureAuto.Set(NeoAPI::ExposureAuto::Off);
			m_impl->m_camera.f().ExposureTime.Set(m_baumerConfigs.exposureTime);
		}

		// print resolution
		m_imgWidth = static_cast<int>(m_impl->m_camera.f().Width);
		m_imgHeight = static_cast<int>(m_impl->m_camera.f().Height);
	}
	catch (NeoAPI::NeoException& exc)
	{
		m_kLogger->error("error: {}", exc.GetDescription());
	}
	catch (std::exception& e)
	{
		m_kLogger->error("error : {}", e.what());
	}

	m_frame.colorImage = cv::Mat::zeros(cv::Size(m_imgWidth, m_imgHeight), CV_8UC3);
	m_frame.depthImage = cv::Mat::zeros(cv::Size(m_imgWidth, m_imgHeight), CV_32FC1);
	m_kLogger->info("Resolution of BaumerCam (w x h): {} x {}", m_imgWidth, m_imgHeight);
}

BaumerCam::~BaumerCam()
{
	this->close();

	m_kLogger->info("Finish BaumerCam");
}

VsFrame BaumerCam::vsGetFrame()
{
	cv::Mat img;
	try
	{
		constexpr uint32_t timeoutMSec = 200;
		NeoAPI::Image image = m_impl->m_camera.GetImage(timeoutMSec);
		if (image.IsEmpty())
		{
			std::cerr << "NeoAPI::Image is empty!" << std::endl;
		}
		else
		{
			if (m_colorDesired)
			{
				img = cv::Mat(cv::Size(m_imgWidth, m_imgHeight), CV_8UC3, image.GetImageData(), cv::Mat::AUTO_STEP);
			}
			else
			{
				img = cv::Mat(cv::Size(m_imgWidth, m_imgHeight), CV_8UC1, image.GetImageData(), cv::Mat::AUTO_STEP);
			}
		}
	}
	catch (NeoAPI::NeoException& exc)
	{
		m_kLogger->error("error: {}", exc.GetDescription());
	}
	catch (std::exception& e)
	{
		m_kLogger->error("error : {}", e.what());
	}

	m_frame.colorImage = img;
	m_frame.depthImage = cv::Mat::zeros(img.size(), CV_32FC1);
	m_frame.frameCounter = m_myFrameID++;
	if (m_frame.colorImage.empty() || m_frame.depthImage.empty())
	{
		m_kLogger->info("Acquiring frame was not successful!");
		return VsFrame();
	}

	if (m_frame.colorImage.channels() < 3)
	{
		cv::cvtColor(m_frame.colorImage, m_frame.colorImage, cv::COLOR_GRAY2BGR);
	}
	// if everything worked as expected, we return the images
	return m_frame;
}

bool BaumerCam::init(TriggerMode triggerMode)
{
	if (triggerMode == TriggerMode::TriggerHardware)
	{
		m_kLogger->debug("TriggeMode for Baumer is TriggerHardware.");
		m_impl->m_camera.f().TriggerActivation.Set(NeoAPI::TriggerActivation::FallingEdge);
		m_impl->m_camera.f().TriggerSource.Set(NeoAPI::TriggerSource::Line0);
		m_impl->m_camera.f().TriggerSelector.Set(NeoAPI::TriggerSelector::FrameStart);
		m_impl->m_camera.f().TriggerMode.Set(NeoAPI::TriggerMode::On);
	}
	else
	{
		m_kLogger->debug("TriggeMode for Baumer is off.");
		m_impl->m_camera.f().TriggerMode.Set(NeoAPI::TriggerMode::Off);

		NeoAPI::Image image;
		while (image.IsEmpty())
		{
			std::cout << "checking if camera delivers imgs" << std::endl;
			constexpr uint32_t timeoutMSec = 200;
			image = m_impl->m_camera.GetImage(timeoutMSec);
		}
	}
	return true;
}

int BaumerCam::close()
{
	m_impl->m_camera.f().TriggerMode.Set(NeoAPI::TriggerMode::Off);
	return 0;
}

VsCameraIntrinsics BaumerCam::vsGetCameraMatrix() { return this->m_cameraMatrix; }

cv::Mat_<double> BaumerCam::getExtrinsics() { return this->m_extrinsics; }

size_t BaumerCam::getImgWidth() { return m_imgWidth; }

size_t BaumerCam::getImgHeight() { return m_imgHeight; }
