/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 */

#include "poseEstimationIPA.h"

#include "poseEstimation.h"
#include "mathHelperFuncs.h"

// needed ipa structs for configs
namespace pickco
{
class GrippingPointDetection;
struct SegmentationRegion
{
	float fWallWidth = 0.0f;   // optional witdth of a bin which allows to ignore it
	std::vector<float> vdims;  // 3 entries with the dimension of the region in x,yz
	cv::Mat_<float> mpose;     // 4x4 matrix transformation betweeen the bottom-center
	                           // coordinates of the segmentation region and the robot in the robot frame
};
}  // namespace pickco

// ipa api
class GraspPoseEstimation
{
  public:
	GraspPoseEstimation(const fs::path& configPath);
	~GraspPoseEstimation();

	std::vector<VSPose> run(const VsCameraIntrinsics& intrinsics,
	                        const cv::Mat_<double>& extrinsics,
	                        const cv::Mat_<float>& depthImg,
	                        const pickco::SegmentationRegion& regSegmentation,
	                        cv::Mat& debugImg);

  private:
	std::unique_ptr<pickco::GrippingPointDetection> p_GPD;
};

class PoseEstimationIPA::Impl
{
  public:
	Impl()
	{
		// bc.fWallWidth = 0.01;
		// bc.vdims = Eigen::Vector3f(0.22, 0.15, 0.2);
		m_bc.fWallWidth = 0.02;
		m_bc.vdims = std::vector<float>{0.2, 0.2, 0.1};
		m_bc.mpose = cv::Mat(4, 4, CV_32F);
		m_bc.mpose << 1, 0, 0, -0.1,  //
		    0, -1, 0, 0,              //
		    0, 0, -1, 0.01,           // 0,//
		    0, 0, 0, 1;
	};
	~Impl() = default;
	std::unique_ptr<GraspPoseEstimation> m_estimator;
	pickco::SegmentationRegion m_bc;
	int m_r_from = 0;
	int m_r_to = 0;
	int m_c_from = 0;
	int m_c_to = 0;
};

PoseEstimationIPA::PoseEstimationIPA(const fs::path& networksConfigPath) : m_impl(std::make_unique<Impl>())
{
	nlohmann::json jsonFile = utils::loadJsonFile(networksConfigPath);
	m_impl->m_bc.fWallWidth = jsonFile.at("appConfigs").at("poseEstimationIPA").at("fWallWidth").get<float>();
	m_impl->m_bc.vdims = jsonFile.at("appConfigs").at("poseEstimationIPA").at("vdims").get<std::vector<float>>();
	std::vector<float> poseData = jsonFile.at("appConfigs").at("poseEstimationIPA").at("mpose").get<std::vector<float>>();
	m_impl->m_bc.mpose =
	    cv::Mat(poseData, true).reshape(1, 4);  // copy data form vector and reshape with one channel and 4 rows

	std::cout << m_impl->m_bc.mpose << "\n";

	m_impl->m_r_from = jsonFile.at("appConfigs").at("poseEstimationIPA").at("r_from").get<int>();
	m_impl->m_r_to = jsonFile.at("appConfigs").at("poseEstimationIPA").at("r_to").get<int>();
	m_impl->m_c_from = jsonFile.at("appConfigs").at("poseEstimationIPA").at("c_from").get<int>();
	m_impl->m_c_to = jsonFile.at("appConfigs").at("poseEstimationIPA").at("c_to").get<int>();
	std::string parameterFilePath = jsonFile.at("appConfigs").at("poseEstimationIPA").at("parameterFile").get<std::string>();
	m_impl->m_estimator = std::make_unique<GraspPoseEstimation>(parameterFilePath);
}

PoseEstimationIPA::~PoseEstimationIPA() = default;

std::vector<VSPose> PoseEstimationIPA::getPoses(const VsCameraIntrinsics& intrinsics,
                                                const cv::Mat_<double>& extrinsics,
                                                const cv::Mat_<float>& depthImg,
                                                cv::Mat& debugImg)
{
	// Cropping: Here 640 x 480
	cv::Mat depth = depthImg.clone();  // TODO(aschaefer): try to remove extra copy
	cv::Mat cropped_depth =
	    depth(cv::Range(m_impl->m_r_from, m_impl->m_r_to), cv::Range(m_impl->m_c_from, m_impl->m_c_to));
	
	constexpr float depthRepacementValue = 1.115f;
	cropped_depth = cv::min(cropped_depth, depthRepacementValue); 
	cv::patchNaNs(cropped_depth, depthRepacementValue);
	for (int v = 0; v < cropped_depth.cols; v++) {
		for (int u = 0; u < cropped_depth.rows; u++) {
			if (cropped_depth.at<float>(v, u) < 0.505) {
				cropped_depth.at<float>(v, u) = depthRepacementValue;
			}
		}
	}

	// Adjust intrinsic: shift principal point to new image center (depends on starting cols)
	VsCameraIntrinsics intrinsicCrop;
	intrinsicCrop = intrinsics;
	intrinsicCrop.cx = intrinsics.cx - (float)m_impl->m_c_from;
	intrinsicCrop.cy = intrinsics.cy - (float)m_impl->m_r_from;
	// Formula above correct?

	cv::Mat extrinsicsIPA = cv::Mat_<double>::zeros(4,4);
	extrinsics.copyTo(extrinsicsIPA(cv::Rect(0,0,4,3)));
        extrinsicsIPA.at<double>(3,3) = 1.0;

 	std::vector<VSPose> poses =
	    m_impl->m_estimator->run(intrinsicCrop, extrinsicsIPA, cropped_depth, m_impl->m_bc, debugImg);
	std::cout << "poses.size() = " << poses.size() << std::endl;
	
	std::vector<VSPose> bestPoses;
	if (poses.size() > 1)
	{
		std::copy(poses.begin(), poses.begin() + 1, back_inserter(bestPoses)); 
		std::cout << "new  bestPoses.size() = " << bestPoses.size() << std::endl;
	}else{
		bestPoses = std::move(poses);	
	}

	// draw poses
	for (auto& pose : bestPoses)
	{
		VSPose debugPose = pose;
		debugPose.x = pose.x;
		debugPose.y = pose.y;
		debugPose.z = pose.z;
		debugPose.roll = 0;
		debugPose.pitch = 0;
		debugPose.yaw = 0;
		
		PoseEstimation::drawPoseRobotCoords(debugPose, intrinsics, extrinsics, debugImg);
	    //cv::Mat rotMat = math::convertPoseEuler2Mat(pose);
		//PoseEstimation::drawPoseCamCoords(pose, rotMat, debugImg,intrinsics);
		//pose = PoseEstimation::makeRobotPose(pose, rotMat, extrinsics);
	}

	return bestPoses;
}
