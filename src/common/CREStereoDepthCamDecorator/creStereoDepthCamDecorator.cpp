#include "creStereoDepthCamDecorator.h"

#include "miscellaneous.h"
#include "projectPaths.h"
#include "stereoDepth.h"

class CreStereoCamDecorator::Impl
{
  public:
	explicit Impl(const fs::path& networkConfigPath, const size_t networkConfigIdx) : m_stereoDepth(std::make_unique<StereoDepth>(networkConfigPath, networkConfigIdx))
	{
		nlohmann::json jsonFile = utils::loadJsonFile(networkConfigPath);
	};
	~Impl() = default;
	std::unique_ptr<StereoDepth> m_stereoDepth;
	std::vector<cv::Mat> m_masks;
};

CreStereoCamDecorator::CreStereoCamDecorator(const size_t networkConfigIdx,
											 std::vector<std::shared_ptr<VsCameraInterface>>& cameras,
                                             const fs::path& networkConfigPath,
                                             const std::vector<std::string> serialNumbers)
    : m_cameras(cameras)
{
	// cameras.at(0) is the camera reference frame for depth information
	m_imgWidth = m_cameras.at(0)->getImgWidth();    // to make sure that getter functions deliver the right values
	m_imgHeight = m_cameras.at(0)->getImgHeight();  // to make sure that getter functions deliver the right values
	m_cameraMatrix = m_cameras.at(0)->vsGetCameraMatrix();  // use the same intrinsic parameters
	m_extrinsics = m_cameras.at(0)->getExtrinsics();
	m_serialNumber = m_cameras.at(0)->getSerialNumber();

	m_impl = std::make_unique<Impl>(networkConfigPath, networkConfigIdx);  // imgSize is needed for the detection area masking
	for (size_t i = 0; i < serialNumbers.size(); ++i)
	{
		cv::Mat mask =
		    misc::getDetectionAreaMask(networkConfigPath,
		                               serialNumbers.at(i),
		                               cv::Size(m_cameras.at(i)->getImgWidth(), m_cameras.at(i)->getImgHeight()),
		                               m_kLogger);
		m_impl->m_masks.emplace_back(mask);
	}
}

CreStereoCamDecorator::~CreStereoCamDecorator() = default;

VsFrame CreStereoCamDecorator::vsGetFrame()
{
	std::vector<VsFrame> frames(m_cameras.size());
	for (size_t i = 0; i < m_cameras.size(); ++i)
	{
		frames.at(i) = m_cameras.at(i)->vsGetFrame();
	}

	if (frames.at(0).colorImage.empty() || frames.at(1).colorImage.empty())
	{
		m_kLogger->warn("CreStereoCamDecorator: colorImg is empty");
		return frames.at(0);
	}
	
	// std::vector<cv::Mat> maskedImgs(frames.size());
	// // input img of gan needs to be grayscale but 3 channels
	// for (size_t i = 0; i < frames.size(); ++i)
	// {
	// 	cv::Mat maskedImg;
	// 	cv::bitwise_and(frames.at(i).colorImage, m_impl->m_masks.at(i), maskedImgs.at(i));
	// }

	frames.at(0).depthImage = m_impl->m_stereoDepth->run(frames.at(0).colorImage, frames.at(1).colorImage);
	// frames.at(0).depthImage.resize(cv::Size(), );

	misc::showImage("creStereoOutput", frames.at(0).depthImage);

	return frames.at(0);
};

int CreStereoCamDecorator::close() { return 0; };

bool CreStereoCamDecorator::init(TriggerMode triggerMode)
{
	bool ret = false;
	for (size_t i = 0; i < m_cameras.size(); ++i)
	{
		ret = m_cameras.at(i)->init(triggerMode);  // make init of HDV or IDS or Mock
		if (ret == false)
		{
			return ret;  // if there is an error then we return
		}
	}
	return ret;
}

VsCameraIntrinsics CreStereoCamDecorator::vsGetCameraMatrix() { return this->m_cameraMatrix; }

size_t CreStereoCamDecorator::getImgWidth() { return m_imgWidth; }

size_t CreStereoCamDecorator::getImgHeight() { return m_imgHeight; }

cv::Mat_<double> CreStereoCamDecorator::getExtrinsics() { return this->m_extrinsics; }
