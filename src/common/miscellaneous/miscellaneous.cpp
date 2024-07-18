#include "miscellaneous.h"

#include <thread>

namespace misc
{
void getDetectionAreaPolygon(std::vector<cv::Point>& polygonDetectArea, const fs::path& visionSytemConfigPath)
{
	std::ifstream ifs(visionSytemConfigPath.string());
	if (!ifs.good())
	{
		throw std::runtime_error("cannot open config file! visionSytemConfigPath = " + visionSytemConfigPath.string());
	}
	nlohmann::json jsonFile = nlohmann::json::parse(ifs);
	try
	{
		for (const auto& item : jsonFile["polygonPoints"])
		{
			int u = item.at("u").get<int>();
			int v = item.at("v").get<int>();
			polygonDetectArea.push_back(cv::Point(u, v));
		}
	}
	catch (const std::exception& e)
	{
		throw std::runtime_error("Could not read parameters from file: " + visionSytemConfigPath.string());
	}	
	ifs.close();
}

cv::Mat getDetectionAreaMask(const fs::path& networksConfigPath,
                             const std::string& serialNumber,
                             const cv::Size& maskSize,
                             const std::shared_ptr<spdlog::logger> logger)
{
	size_t camIdx = utils::getCamIdxFromNetworksConfig(serialNumber, networksConfigPath);
	std::vector<cv::Point> polygonDetectArea;
	nlohmann::json jsonFile = utils::loadJsonFile(networksConfigPath);
	try
	{
		for (const auto& item : jsonFile.at("cameraConfigs").at("camera")[camIdx]["polygonPoints"])
			{
				int u = item.at("u").get<int>();
				int v = item.at("v").get<int>();
				polygonDetectArea.push_back(cv::Point(u, v));
			}
	}
	catch (const std::exception& e)
	{
		logger->error("Could not read parameters from file: " + networksConfigPath.string());
	}
	
	cv::Mat maskDetectionArea = cv::Mat::zeros(maskSize, CV_8UC3);
	logger->debug("maskSize = {}", maskSize);
	logger->debug("maskDetectionArea = {}", maskDetectionArea.size());
	// drawing a white area inside the polygon from the config file
	cv::fillConvexPoly(maskDetectionArea,
	                   polygonDetectArea,
	                   cv::Scalar(255, 255, 255));  // needs to be 255 because later we do bitwise_and
	return maskDetectionArea;
}

void saveDetectionsToDisk(const std::vector<VSPose>& posesInRobotCoordinates,
                          const VsFrame& frame,
                          const uint64_t frameNumber,
                          const std::shared_ptr<spdlog::logger> logger)
{
	logger->debug("pose = {}, {}, {}, {}, {},{}",
	              posesInRobotCoordinates[0].x,
	              posesInRobotCoordinates[0].y,
	              posesInRobotCoordinates[0].z,
	              posesInRobotCoordinates[0].roll,
	              posesInRobotCoordinates[0].pitch,
	              posesInRobotCoordinates[0].yaw);
	std::string depthFileName = "recorded/depthImages/depthImage_" + std::to_string(frameNumber) + ".tiff";
	cv::imwrite(depthFileName, frame.depthImage);
	std::string colorFileName = "recorded/colorImages/colorImage_" + std::to_string(frameNumber) + ".png";
	cv::imwrite(colorFileName, frame.colorImage);
	std::string poseFilename = "recorded/detections/poseStorage_" + std::to_string(frameNumber) + ".yml";
	cv::FileStorage fs(poseFilename, cv::FileStorage::WRITE);
	fs << "number_of_poses" << (int)posesInRobotCoordinates.size();
	fs << "posesInRobotCoordinates"
	   << "[";
	for (size_t i = 0; i < posesInRobotCoordinates.size(); ++i)
	{
		fs << "{";
		fs << "x" << posesInRobotCoordinates[i].x;
		fs << "y" << posesInRobotCoordinates[i].y;
		fs << "z" << posesInRobotCoordinates[i].z;
		fs << "roll" << posesInRobotCoordinates[i].roll;
		fs << "pitch" << posesInRobotCoordinates[i].pitch;
		fs << "yaw" << posesInRobotCoordinates[i].yaw;
		fs << "}";
	}
	fs << "]";
	fs.release();
}

bool enableVisualization = true;

void setVisualization(bool enableFlag) { enableVisualization = enableFlag; }

void waitKey1ms()
{
	if (enableVisualization)
	{
		cv::waitKey(1);
	}
}

void showImage(const std::string& caption, const cv::Mat& src)
{
	if (enableVisualization)
	{
		if (src.empty())
		{
			return;
		}
		std::stringstream ss;
		ss << std::this_thread::get_id();
		std::string captionWithThreadId = caption + ss.str();
		try
		{
			cv::namedWindow(captionWithThreadId, cv::WINDOW_NORMAL);
			cv::imshow(captionWithThreadId, src);
		}
		catch (const std::exception& e)
		{
			std::cerr << e.what() << '\n';
		}
	}
}

void getFrame(VsFrame& frame, cv::Mat& debugResImg, const std::shared_ptr<VsCameraInterface> camera)
{
	frame = camera->vsGetFrame();

	if (frame.colorImage.empty())
	{
		return;
	}

	debugResImg = frame.colorImage.clone();  // later we draw all the detected poses to the resultImage
	misc::showImage("sourceImage", frame.colorImage);
	misc::showImage("depth", frame.depthImage);
}

void stechtestPoseSelection(size_t& poseIdx, std::vector<VSPose>& posesInRobotCoordinates, std::vector<int>& classIds)
{
	if (poseIdx >= posesInRobotCoordinates.size())
	{
		poseIdx = 0;  // if iterated through all poses start with the first again
	}

	std::sort(posesInRobotCoordinates.begin(), posesInRobotCoordinates.end(), [](const VSPose& a, const VSPose& b) {
		return (a.x < b.x) || (a.x == b.x && a.y < b.y);
	});

	VSPose stechtestPose = posesInRobotCoordinates.at(poseIdx);
	int stechtestClass = classIds.at(poseIdx);
	posesInRobotCoordinates.clear();
	classIds.clear();
	posesInRobotCoordinates.push_back(stechtestPose);
	classIds.push_back(stechtestClass);
	++poseIdx;  // prepare for next pose
}

void parseCommandLineInput(int argc,
                           char* argv[],
                           fs::path& cameraConfigPath,
                           fs::path& visionSystemConfigPath,
                           fs::path& formatConfigPath,
                           fs::path& networksConfigPath)
{
	// overwriting the default values with the command line arguments/values
	std::string keys = "{cameraConfigPath   |                 | camera configuration path}"   // default value ""
	                   "{networksConfigPath   |      | networks configuration path}"          // default value ""
	                   "{visionSystemConfigPath  |      | vision system configuration path}"  // default value ""
	                   "{formatConfigPath  |      | format configuration path}"               // default value ""
	                   "{help   |      | show help message}";

	cv::CommandLineParser parser(argc, argv, keys);
	if (parser.has("help"))
	{
		parser.printMessage();
		exit(-1);
	}
	if (parser.has("cameraConfigPath"))
	{
		cameraConfigPath = parser.get<std::string>("cameraConfigPath");
	}
	if (parser.has("visionSystemConfigPath"))
	{
		visionSystemConfigPath = parser.get<std::string>("visionSystemConfigPath");
	}
	if (parser.has("formatConfigPath"))
	{
		formatConfigPath = parser.get<std::string>("formatConfigPath");
	}
	if (parser.has("networksConfigPath"))
	{
		networksConfigPath = parser.get<std::string>("networksConfigPath");
	}
}

}  // namespace misc
