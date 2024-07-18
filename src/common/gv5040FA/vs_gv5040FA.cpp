
/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "vs_gv5040FA.h"

#include <fstream>
#include <opencv2/opencv.hpp>
#include <peak/converters/peak_buffer_converter_ipl.hpp>
#include <peak/peak.hpp>
#include <peak_ipl/peak_ipl.hpp>

#include "json.hpp"
#include "projectPaths.h"

class VsGV5040FA::Impl
{
  public:
	Impl() { peak::Library::Initialize(); };
	~Impl() = default;
	std::shared_ptr<peak::core::DataStream> m_dataStream;
	std::shared_ptr<peak::core::Device> m_device;
	std::shared_ptr<peak::core::NodeMap> m_nodeMapRemoteDevice;
};

VsGV5040FA::VsGV5040FA(const fs::path& cameraConfigPath,
                       const fs::path& networksConfigPath,
                       const std::string& serialNumber,
                       bool maxResolution)
    : m_impl(std::make_unique<Impl>())
{
	int camIdx = utils::getCamIdxFromCameraConfig(serialNumber, cameraConfigPath);
	// read intrinsic paramters from camera.json, needed for poseEstimation when gan is used
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
		return;
	}
	
	
	this->m_serialNumber = serialNumber;

	camIdx = utils::getCamIdxFromNetworksConfig(serialNumber, networksConfigPath);
	jsonFile = utils::loadJsonFile(networksConfigPath);

	if (maxResolution)  // for hand eye calibration
	{
		m_cropConfigs.cropX = 0;
		m_cropConfigs.cropY = 0;
		constexpr size_t kMaxPixelWidth = 1440;
		constexpr size_t kMaxPixelHeight = 1080;
		m_cropConfigs.cropWidth = kMaxPixelWidth;
		m_cropConfigs.cropHeight = kMaxPixelHeight;
	}
	else
	{
		try
		{
			m_cropConfigs.cropX = jsonFile.at("cameraConfigs").at("camera")[camIdx].at("cropX").get<size_t>();
			m_cropConfigs.cropY = jsonFile.at("cameraConfigs").at("camera")[camIdx].at("cropY").get<size_t>();
			m_cropConfigs.cropWidth = jsonFile.at("cameraConfigs").at("camera")[camIdx].at("cropWidth").get<size_t>();
			m_cropConfigs.cropHeight = jsonFile.at("cameraConfigs").at("camera")[camIdx].at("cropHeight").get<size_t>();
		}
		catch (const std::exception& e)
		{
			m_kLogger->error("Could not read parameters from file: " + networksConfigPath.string());
			return;
		}	
	}
	try
	{
			m_colorDesired = jsonFile.at("cameraConfigs").at("camera")[camIdx].at("color").get<bool>();
			m_idsConfigs.exposureTime = jsonFile.at("cameraConfigs").at("camera")[camIdx].at("exposureTime").get<int>();
	}
	catch (const std::exception& e)
	{
		m_kLogger->error("Could not read parameters from file: " + networksConfigPath.string());
		return;
	}
	

	fs::path idsConfigPath = utils::getProjectRootDir() / "config/gv5040FAConfig.json";
	jsonFile = utils::loadJsonFile(idsConfigPath);
	try
	{
		m_idsConfigs.gain = jsonFile.at("gain").get<float>();
	}
	catch (const std::exception& e)
	{
		m_kLogger->error("Could not read parameters from file: " + idsConfigPath.string());
		return;
	}
	m_configured = true;
	
	// create a device manager object
	peak::DeviceManager& deviceManager = peak::DeviceManager::Instance();
	// update the deviceManager
	deviceManager.Update();

	// exit program if no device was found
	if (deviceManager.Devices().empty())
	{
		// close peak library
		close();
		throw std::runtime_error("GV5040FA: No device found. Exiting.");
		return;
	}

	// list all available devices
	m_kLogger->info("Device Serial number from config file: {}", m_serialNumber);
	size_t i = 0;
	size_t selectedDevice = -1;
	for (const auto& deviceDescriptor : deviceManager.Devices())
	{
		m_kLogger->info("Device available: {} ({}; {} v.{}, serNo: {})",
		                deviceDescriptor->ModelName(),
		                deviceDescriptor->ParentInterface()->DisplayName(),
		                deviceDescriptor->ParentInterface()->ParentSystem()->DisplayName(),
		                deviceDescriptor->ParentInterface()->ParentSystem()->Version(),
		                deviceDescriptor->SerialNumber());

		if (deviceDescriptor->SerialNumber() == m_serialNumber)
		{
			selectedDevice = i;
		}
		++i;
	}

	if (selectedDevice == -1)
	{
		throw std::runtime_error("ERROR : No IDS GV-504xFA-C camera was found");
		return;
	}

	// open the selected device
	m_impl->m_device = deviceManager.Devices().at(selectedDevice)->OpenDevice(peak::core::DeviceAccessType::Control);

	if (!m_impl->m_device)
	{
		m_kLogger->error("Device could not be opened");
	}
	// Open standard data stream
	m_impl->m_dataStream = m_impl->m_device->DataStreams().at(0)->OpenDataStream();

	// get the remote device node map
	m_impl->m_nodeMapRemoteDevice = m_impl->m_device->RemoteDevice()->NodeMaps().at(0);

	// print model name and user ID
	m_kLogger->info("Model Name: {}",
	                m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::StringNode>("DeviceModelName")->Value());
	m_kLogger->info("User ID: {}",
	                m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::StringNode>("DeviceUserID")->Value());

	try
	{
		// print sensor information, not knowing if device has the node "SensorName"
		m_kLogger->info("Sensor Name: {}",
		                m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::StringNode>("SensorName")->Value());

		m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("UserSetSelector")
		    ->SetCurrentEntry("Default");
		//            m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("UserSetSelector")
		//                ->SetCurrentEntry("/home/kobot/Desktop/ids_101/ids_peak/open_camera/camera_settings.cset");
		m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("UserSetLoad")->Execute();
		m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("UserSetLoad")->WaitUntilDone();
	}
	catch (const std::exception& e)
	{
		// if SensorName is not a valid node name, do error handling here...
		m_kLogger->warn("Caught exception: {}", e.what());
		return;
	}

	// Set ROI
	try
	{
		m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Width")->SetValue(
		    m_cropConfigs.cropWidth);
		m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Height")->SetValue(
		    m_cropConfigs.cropHeight);
		m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("OffsetX")->SetValue(
		    m_cropConfigs.cropX);
		m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("OffsetY")->SetValue(
		    m_cropConfigs.cropY);
		m_kLogger->debug("correcting intrinsics according to crop of img");
		this->m_cameraMatrix.cx = this->m_cameraMatrix.cx - m_cropConfigs.cropX;
		this->m_cameraMatrix.cy = this->m_cameraMatrix.cy - m_cropConfigs.cropY;
	}
	catch (const std::exception& e)
	{
		// if ROI setting fails
		m_kLogger->warn("can not set camera ROI: {}", e.what());
		return;
	}

	// print resolution
	m_imgWidth = m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Width")->Value();
	m_imgHeight = m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Height")->Value();

	m_frame.colorImage = cv::Mat::zeros(cv::Size(m_imgWidth, m_imgHeight), CV_8UC3);
	m_frame.depthImage = cv::Mat::zeros(cv::Size(m_imgWidth, m_imgHeight), CV_32FC1);
	m_kLogger->info("Resolution of GV5040FA (w x h): {} x {}", m_imgWidth, m_imgHeight);
}

VsGV5040FA::~VsGV5040FA()
{
	this->close();

	m_kLogger->info("Finish VsGV5040FA");
}

bool VsGV5040FA::init(TriggerMode triggerMode)
{
	m_triggerMode = triggerMode;
	try
	{
		m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("ExposureTime")
		    ->SetValue(m_idsConfigs.exposureTime);
		m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")
		    ->SetCurrentEntry("AnalogAll");
		m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gain")->SetValue(m_idsConfigs.gain);

		// Set PixelFormat to "BayerRG8"
		if (m_colorDesired)
		{
			m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PixelFormat")
			    ->SetCurrentEntry("RGB8");
		}
		else
		{
			m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PixelFormat")
			    ->SetCurrentEntry("Mono8");
		}

		// Set ExposureTime to "Continuous"
		m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("ExposureAuto")
		    ->SetCurrentEntry("Off");
		// Set GainAuto to "Continuous"
		m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainAuto")->SetCurrentEntry("Off");
		// Set ColorCorrectionMode to "Auto"
		m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("ColorCorrectionMode")
		    ->SetCurrentEntry("Off");

		// Get the payload size for correct buffer allocation
		auto payloadSize =
		    m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("PayloadSize")->Value();

		// Get the minimum number of buffers that must be announced
		auto bufferCountMax = m_impl->m_dataStream->NumBuffersAnnouncedMinRequired();
		m_kLogger->info("bufferCountMax = ", bufferCountMax);

		// Allocate and announce image buffers and queue them
		for (size_t bufferCount = 0; bufferCount < bufferCountMax; ++bufferCount)
		{
			auto buffer = m_impl->m_dataStream->AllocAndAnnounceBuffer(static_cast<size_t>(payloadSize), nullptr);
			m_impl->m_dataStream->QueueBuffer(buffer);
		}

		if (m_triggerMode == TriggerMode::TriggerOff)
		{
			m_kLogger->info("set TriggerMode of GV5040FA to Software");
			m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerMode")
			    ->SetCurrentEntry("On");
			m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerSource")
			    ->SetCurrentEntry("Software");
		}
		else
		{
			m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerSelector")
			    ->SetCurrentEntry("ExposureStart");
			m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerMode")
			    ->SetCurrentEntry("On");
			m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerSource")
			    ->SetCurrentEntry(
			        "Line0");  // hardware trigger on line 0 (opto IN) --> Line0
			                   // hardware trigger on line 2 (GPIO 1) --> Line2
			                   // hardware trigger on line 3 (GPIO 2) --> Line3
			                   //        nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerSource")
			                   //                            ->SetCurrentEntry("Software");
			m_kLogger->info("TriggerActivation");
			m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerActivation")
			    ->SetCurrentEntry("FallingEdge");
		}

		m_kLogger->info("TLParamsLocked");
		// Lock critical features to prevent them from changing during acquisition
		m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("TLParamsLocked")->SetValue(1);

		// Start acquisition
		m_kLogger->info("StartAcquisition");
		m_impl->m_dataStream->StartAcquisition();
		m_kLogger->info("Execute");
		m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStart")->Execute();
		m_kLogger->info("WaitUntilDone");
		m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStart")->WaitUntilDone();
	}
	catch (const std::exception& e)
	{
		m_kLogger->warn("Caught exception: {}", e.what());
		return false;
	}

	return true;
}

int VsGV5040FA::close()
{
	if (m_impl->m_dataStream)
	{
		try
		{
			m_impl->m_dataStream->KillWait();  //->KillOneWait();
			m_impl->m_dataStream->StopAcquisition(peak::core::AcquisitionStopMode::Default);
			m_impl->m_dataStream->Flush(peak::core::DataStreamFlushMode::DiscardAll);

			for (const auto& buffer : m_impl->m_dataStream->AnnouncedBuffers())
			{
				m_impl->m_dataStream->RevokeBuffer(buffer);
			}
		}
		catch (const std::exception& e)
		{
			m_kLogger->warn("Exception caught in StopAcquisition: {}", e.what());
		}
	}
	// close peak library
	peak::Library::Close();

	return 0;
}

VsFrame VsGV5040FA::vsGetFrame()
{
	if (m_triggerMode == TriggerMode::TriggerOff)  // for hand eye calibration
	{
		m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("TriggerSoftware")->Execute();
		m_kLogger->info("WaitUntilDone");
		m_impl->m_nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("TriggerSoftware")->WaitUntilDone();
	}

	cv::Mat img_cv;
	if (m_colorDesired)
	{
		img_cv = cv::Mat::zeros(cv::Size(m_imgWidth, m_imgHeight), CV_8UC3);
	}
	else
	{
		img_cv = cv::Mat::zeros(cv::Size(m_imgWidth, m_imgHeight), CV_8UC1);
	}
	size_t sizeInBytes = img_cv.step[0] * img_cv.rows;

	// Get buffer from device's datastream within given timeout in ms
	std::shared_ptr<peak::core::Buffer> buffer = nullptr;
	try
	{
		buffer = m_impl->m_dataStream->WaitForFinishedBuffer(1000);
	}
	catch (std::exception& e)
	{
		m_kLogger->warn(e.what());
		m_kLogger->warn("GV5040FA WaitForFinishedBuffer exception!");
		return VsFrame();
	}

	if (buffer == nullptr || !buffer->HasNewData())
	{
		m_kLogger->info("GV5040FA has no new data!");
		return VsFrame();
	}
	// Create IDS peak IPL image for debayering and convert it to BGR8 format

	peak::ipl::Image image;
	if (m_colorDesired)
	{
		image = peak::BufferTo<peak::ipl::Image>(buffer).ConvertTo(
		    peak::ipl::PixelFormatName::BGR8, img_cv.data, sizeInBytes);
	}
	else
	{
		image = peak::BufferTo<peak::ipl::Image>(buffer).ConvertTo(
		    peak::ipl::PixelFormatName::Mono8, img_cv.data, sizeInBytes);
	}

	// Queue buffer so that it can be used again
	m_impl->m_dataStream->QueueBuffer(buffer);

	std::memcpy(img_cv.data, image.Data(), sizeInBytes);

	m_frame.colorImage = img_cv.clone();

	m_frame.depthImage = cv::Mat::zeros(img_cv.size(), CV_32FC1);
	m_frame.frameCounter = m_myFrameID++;

	if (m_frame.colorImage.empty() || m_frame.depthImage.empty())
	{
		m_kLogger->info("Acquiring frame was not successful!");
		return VsFrame();
	}

	// if everything worked as expected, we return the images
	return m_frame;
}

VsCameraIntrinsics VsGV5040FA::vsGetCameraMatrix() { return this->m_cameraMatrix; }

cv::Mat_<double> VsGV5040FA::getExtrinsics() { return this->m_extrinsics; }

size_t VsGV5040FA::getImgWidth() { return m_imgWidth; }

size_t VsGV5040FA::getImgHeight() { return m_imgHeight; }
