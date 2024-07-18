/**
 * @copyright Copyright (c) 2020 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * @brief opecnv poseEstimation class
 *
 */

#include "poseEstimation.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/common/distances.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <fstream>
#include <opencv2/core/eigen.hpp>

#include "external/hungarian-algorithm/Hungarian.h"
#include "json.hpp"
#include "mathHelperFuncs.h"
#include "projectPaths.h"

// Impl class is to avoid pcl headers in poseEstimation.h and reduce dependency chains
class PoseEstimation::Impl
{
  public:
	Impl(){};
	~Impl() = default;

	/**
	 * @brief drawing the 6D pose in the debugImg
	 */
	void drawPose(const VSPose& poseCameraCoords,
	              const cv::Mat& product2CameraTransf,
	              cv::Mat& debugImg,
	              const VsCameraIntrinsics& intrinsics);

	/**
	 * @brief computes the plane parameters via ransac estimation (done on CPU)
	 */
	void planeEstimationCPU(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	                        pcl::ModelCoefficients::Ptr modelCoefficientsCPU);

	/**
	 * @brief computes from depthmap the x,y,z coordinates(point cloud) within a mask in the object
	 * zero values of depthmap are going to be filtered
	 * if the point cloud contains less than 3 valid points, the object should be ignored
	 */
	void getPointcloud(const cv::Mat& mask,
	                   const cv::Mat_<float>& depthImg,
	                   const VsCameraIntrinsics& cMatrix,
	                   pcl::PointCloud<pcl::PointXYZ>::Ptr dst,
					   const PoseConfig& poseConfig) const;

	/**
	 * @brief calculates the x-Axis of the object (read arrow in debug img)
	 * giving the keypoints and the aimed configured direction
	 */
	Eigen::Vector3f calcDirectionVector(const VsCameraIntrinsics& intrinsics,
	                                    const std::vector<cv::Point2f>& keyPoints,
	                                    const PoseConfig& poseConfig,
	                                    const pcl::PointXYZ& centerPoint,
	                                    const Eigen::Vector3f& normalVec,
										const bool pointsFlipped) const;

    /**
	 * @brief calculates the z orientation of an object with performing PCA analysis giving a set of keypoints
	 * along the aimed direction vector of an object
	 * a double check of the orientation with other sets of keypoints is made to ensure that there are no outliers 
	 * in the keypoints
	 */
	float getRotationAngleOfKeyPoints(const std::vector<cv::Point2f>& points, const PoseConfig& poseConfig, const bool pointsFlipped) const;

    /**
	 * @brief calculating the region of the interest where the plane of the object should be estimated
	 * drawing the region also on the debugImg
	 */
	cv::Mat drawMaskFromKeypoints(const cv::Point centerOfMask,
	                              const std::vector<cv::Point2f>& keyPoints,
	                              const cv::Size& imgSize,
	                              const PoseConfig& poseConfig,
	                              cv::Mat& debugImg) const;

	/**
	 * @brief calculating the region of the interest where the plane of the object should be estimated
	 * and the center of this region: call the method: drawMaskFromKeypoints()
	 */						  
	std::tuple<cv::Mat, cv::Point> getMaskAndcenterpoint2d(const std::vector<cv::Point2f>& keyPoints,
	                                                     const cv::Rect& rectDetection,
	                                                     const cv::Size& imgSize,
	                                                     const PoseConfig& poseConfig,
	                                                     cv::Mat& debugImg) const;

    /**
	 * @brief calculating the rotation matrix of the object from direction vector and normal vector
	 * the left vector gets calculated as a cross product from direction and normal vectors
	 */	
	cv::Mat_<float> buildRotationMatrix(Eigen::Vector3f& directionVec, Eigen::Vector3f& normalVec);

	/**
	 * @brief uses the x,y,z and the rotation Matrix (in camera coordinates) to calculate the pose in robot coordinates
	 * can fix here also some degrees of freedom
	 */
	VSPose makeRobotPose(const VSPose& centerOfProduct, const cv::Mat& rotMat, const cv::Mat& extrinsics);

    /**
	 * @brief calculates the 6D pose of an object
	 */
	VSPose calcPoseWithKeyPoints(const VsCameraIntrinsics& intrinsics,
	                             const cv::Mat_<double>& extrinsics,
	                             const cv::Mat_<float>& depthImg,
	                             const float constZInRobotCoords,
	                             const std::vector<cv::Point2f>& keyPoints,
	                             const PoseConfig& poseConfig,
	                             const cv::Rect& rectDetection,
	                             const int classId,
	                             cv::Mat& debugImg,
								 const bool pointsFlipped);
    /**
	 * @brief calculates x,y,z and the normal vector of an object
	 */
	void calccenterpoint3d(const cv::Mat_<float>& depthImg,
	                     const float constZInRobotCoords,
	                     const cv::Mat& maskWithBigContour,
	                     const cv::Point& centerOfMask,
	                     const VsCameraIntrinsics& intrinsics,
	                     const cv::Mat_<double>& extrinsics,
	                     pcl::PointXYZ& centerpoint3d,
	                     Eigen::Vector3f& normalVec,
						 const PoseConfig& poseConfig);

	/**
	 * @brief for homogenous object we can not determine the orientation around z axis within keypoints
	 * for that we use the keypoints and measure the dimension of an edge ==> determine the orientation
	 */
	void calcXAxisWithLongShortEdge(const VsCameraIntrinsics& intrinsics,
									const cv::Mat_<double>& extrinsics,
	                                const cv::Mat_<float>& depthImg,
	                                const int classId,
	                                std::vector<cv::Point2f>& keyPoints,
	                                PoseConfig& poseConfig,
	                                cv::Mat& debugImg,
									bool& pointsFlipped);

	/**
	 * @brief giving a set of keypoints and normal distances between those keypoints, check if these distances
	 * are correct. If not the class of the object turns to bad
	 */
	bool checkDimQualityGate(const VsCameraIntrinsics& intrinsics,
	                         const cv::Mat_<double>& extrinsics,
	                         const cv::Mat_<float>& depthImg,
	                         const int classId,
	                         const std::vector<cv::Point2f>& keyPoints,
	                         PoseConfig& poseConfig,
	                         cv::Mat& debugImg);

	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("poseEstimation");

	cv::Mat m_rotMat;
	int m_areaNumber = 1;
	// for all defined dlcs we need the keypoint specifications
	std::vector<std::vector<unsigned int>> m_keyPointIdxsXAxis = {};
	std::vector<std::vector<unsigned int>> m_keyPointIdxsMaskForDepth = {};
	std::vector<std::vector<AxisVecIdxs>> m_xAxisDoubleCheckerIdxs = {};
	std::vector<centerpointCalcMethod> m_centerpointCalcMethod = {};
	std::vector<int> m_centerpointIdx = {};
	bool m_fixRollAngle = false;
	bool m_fixPitchAngle = false;
	bool m_fixYawAngle = false;
	std::map<int, int> m_classToDlcIdxMapping;
	std::map<int, float> m_classToValuesInMeterMapping;
	bool m_constDepthEnable = false;
	std::vector<int> m_loopOffsetDepthPoints = {};
	std::vector<PoseConfig> m_poseConfigs;  // for all defined dlcs we need the keypoint specifications
};

PoseEstimation::PoseEstimation(const fs::path& networksConfigPath) : m_impl(std::make_unique<Impl>())
{
	nlohmann::json jsonFile = utils::loadJsonFile(networksConfigPath);
	try
	{
		m_impl->m_areaNumber = jsonFile.at("areaNumber").get<int>();
		for (auto it = jsonFile.at("appConfigs").at("yolo").at("detectionConfigs").at("classToDLCIndexMapping").begin();
			it != jsonFile.at("appConfigs").at("yolo").at("detectionConfigs").at("classToDLCIndexMapping").end();
			++it)
		{
			m_impl->m_classToDlcIdxMapping[std::stoi(it.key())] = it.value();
		}

		m_impl->m_constDepthEnable = jsonFile.at("appConfigs").at("yolo").at("detectionConfigs").at("desiredClassesConstZ").at("enable").get<bool>();
		for (auto it = jsonFile.at("appConfigs").at("yolo").at("detectionConfigs").at("desiredClassesConstZ").at("classToValuesInMeterMapping").begin();
			it != jsonFile.at("appConfigs").at("yolo").at("detectionConfigs").at("desiredClassesConstZ").at("classToValuesInMeterMapping").end();
			++it)
		{
			m_impl->m_classToValuesInMeterMapping[std::stoi(it.key())] = it.value();
		}
		m_impl->m_poseConfigs.clear();
		int loopOffsetDepthPoints = 1;
		m_impl->m_loopOffsetDepthPoints.clear();
		m_impl->m_loopOffsetDepthPoints.emplace_back(loopOffsetDepthPoints);
		if(m_impl->m_areaNumber == 1 || m_impl->m_areaNumber == 2) // This is needed for areas where we are going to pick products
		{
			
			if(jsonFile.at("appConfigs").at("generalConfigs").at("pickMode").get<std::string>() == "conveyor")
			{
				for (int i = 0; i < jsonFile.at("appConfigs").at("dlcs").size(); ++i)
				{
					m_impl->m_poseConfigs.emplace_back(PoseConfig());
					// get keypoint idxs, which are needed for orientation calculation
					for (const auto& item : jsonFile.at("appConfigs").at("dlcs").at(i).at("poseEstimationConfigs").at("orientationZConfigs").at("keypointIndicesXAxis"))
					{
						m_impl->m_poseConfigs.at(i).m_keyPointIdxsXAxis.emplace_back(item);
					}

					for (const auto& item : jsonFile.at("appConfigs").at("dlcs").at(i).at("poseEstimationConfigs").at("planeEstimationConfigs").at("keypointIndicesMaskForDepth"))
					{
						m_impl->m_poseConfigs.at(i).m_keyPointIdxsMaskForDepth.emplace_back(item);
					}

					for (const auto& item : jsonFile.at("appConfigs").at("dlcs").at(i).at("poseEstimationConfigs").at("orientationZConfigs").at("keypointIndicesXAxisDoubleCheck"))
					{
						AxisVecIdxs vecIdxs;
						vecIdxs.m_headIdx = item.at("vectorHeadIdx").get<size_t>();
						vecIdxs.m_tailIdx = item.at("vectorTailIdx").get<size_t>();
						m_impl->m_poseConfigs.at(i).m_xAxisDoubleCheckerIdxs.emplace_back(vecIdxs);
					}

					std::string centerpointCalculation = jsonFile.at("appConfigs").at("dlcs").at(i).at("poseEstimationConfigs").at("centerpointConfigs").at("centerpoint");
					if (centerpointCalculation == "fromRect")
					{
						m_impl->m_poseConfigs.at(i).m_centerpointCalcMethod = centerpointCalcMethod::FromRect;
					}
					else if (centerpointCalculation == "fromKeyPoint")
					{
						m_impl->m_poseConfigs.at(i).m_centerpointCalcMethod = centerpointCalcMethod::FromKeyPoints;
					}
					for (int j = 0; j < jsonFile.at("appConfigs").at("dlcs").at(i).at("poseEstimationConfigs").at("centerpointConfigs").at("centerpointIdxs").size(); ++j)
					{
						m_impl->m_poseConfigs.at(i).m_centerpointIdxs.emplace_back(
							jsonFile.at("appConfigs").at("dlcs").at(i).at("poseEstimationConfigs").at("centerpointConfigs").at("centerpointIdxs").at(j).get<int>());
					}

					m_impl->m_poseConfigs.at(i).m_longShortEdgePoseEstimationEnable =
						jsonFile.at("appConfigs").at("dlcs").at(i).at("poseEstimationConfigs").at("orientationZConfigs").at("longShortConfigs").at("longShortEdgePoseEstimationEnable");
					m_impl->m_poseConfigs.at(i).m_longShortEdgeLengthInMeterThr =
						jsonFile.at("appConfigs").at("dlcs").at(i).at("poseEstimationConfigs").at("orientationZConfigs").at("longShortConfigs").at("longShortEdgeLengthInMeterThr");
					m_impl->m_poseConfigs.at(i).m_idxOfKeyPointForAxis0 = jsonFile.at("appConfigs").at("dlcs").at(i).at("poseEstimationConfigs").at("orientationZConfigs").at("longShortConfigs").at("idxOfKeyPointForAxis0");
					m_impl->m_poseConfigs.at(i).m_idxOfKeyPointForAxis1 = jsonFile.at("appConfigs").at("dlcs").at(i).at("poseEstimationConfigs").at("orientationZConfigs").at("longShortConfigs").at("idxOfKeyPointForAxis1");

					m_impl->m_poseConfigs.at(i).m_deltaAngleBetweenAxesThresholdRad =
						jsonFile.at("appConfigs").at("dlcs").at(i).at("poseEstimationConfigs").at("orientationZConfigs").at("deltaAngleBetweenAxesThresholdInDegree").get<float>() / 360.0 * 2.0 * CV_PI;

					// Test QualityGateDimension
					m_impl->m_poseConfigs.at(i).m_qualityGateDim.m_enable =
						jsonFile.at("appConfigs").at("dlcs").at(i).at("qualityGateDimension").at("enable");

					for (int j = 0; j < jsonFile.at("appConfigs").at("dlcs").at(i).at("qualityGateDimension").at("idxs").size(); ++j)
					{
						int idx0 = jsonFile.at("appConfigs").at("dlcs").at(i).at("qualityGateDimension").at("idxs").at(j).at("idx0");
						int idx1 = jsonFile.at("appConfigs").at("dlcs").at(i).at("qualityGateDimension").at("idxs").at(j).at("idx1");
						float thr = jsonFile.at("appConfigs").at("dlcs").at(i).at("qualityGateDimension").at("idxs").at(j).at("distanceThrInMeter");
						m_impl->m_poseConfigs.at(i).m_qualityGateDim.m_idxsAndThrs.emplace_back(
							QualityGateDimesion::IdxsAndThr{idx0, idx1, thr});
					}
				
					loopOffsetDepthPoints = jsonFile.at("appConfigs").at("dlcs").at(i).at("poseEstimationConfigs").at("planeEstimationConfigs").at("loopOffsetDepthPoints").get<int>();
					m_impl->m_loopOffsetDepthPoints.emplace_back(loopOffsetDepthPoints);

					m_impl->m_poseConfigs.at(i).m_useConvexHull = jsonFile.at("appConfigs").at("dlcs").at(i).at("poseEstimationConfigs").at("planeEstimationConfigs").at("useConvexHull").get<bool>();
					
					m_impl->m_poseConfigs.at(i).m_maxDepth = jsonFile.at("appConfigs").at("dlcs").at(i).at("poseEstimationConfigs").at("planeEstimationConfigs").at("maxDepth").get<float>();
					m_impl->m_poseConfigs.at(i).m_minDepth = jsonFile.at("appConfigs").at("dlcs").at(i).at("poseEstimationConfigs").at("planeEstimationConfigs").at("minDepth").get<float>();
					
					m_impl->m_poseConfigs.at(i).m_layersConfig.m_useLayers = jsonFile.at("appConfigs").at("dlcs").at(i).at("poseEstimationConfigs").at("planeEstimationConfigs").at("layers").at("useLayers").get<bool>();
					for (int j = 0; j < jsonFile.at("appConfigs").at("dlcs").at(i).at("poseEstimationConfigs").at("planeEstimationConfigs").at("layers").at("layersValues").size(); ++j)
					{
						float layersValue = jsonFile.at("appConfigs").at("dlcs").at(i).at("poseEstimationConfigs").at("planeEstimationConfigs").at("layers").at("layersValues").at(j);
						m_impl->m_poseConfigs.at(i).m_layersConfig.m_layersValues.emplace_back(layersValue);
					}
					m_impl->m_poseConfigs.at(i).m_layersConfig.m_layersResolution = jsonFile.at("appConfigs").at("dlcs").at(i).at("poseEstimationConfigs").at("planeEstimationConfigs").at("layers").at("layersResolution").get<float>();

				}
			}
			m_impl->m_fixRollAngle = jsonFile.at("appConfigs").at("posePostProcessing").at("fixRollAngle").get<bool>();
			m_impl->m_fixPitchAngle = jsonFile.at("appConfigs").at("posePostProcessing").at("fixPitchAngle").get<bool>();
			m_impl->m_fixYawAngle = jsonFile.at("appConfigs").at("posePostProcessing").at("fixYawAngle").get<bool>();
		}
		else
		{
			for (int i = 0; i < jsonFile.at("appConfigs").at("dlcs").size(); ++i)
				{
					m_impl->m_poseConfigs.emplace_back(PoseConfig());
					// get keypoint idxs, which are needed for orientation calculation
					for (const auto& item : jsonFile.at("appConfigs").at("dlcs").at(i).at("poseEstimationConfigs").at("orientationZConfigs").at("keypointIndicesXAxis"))
					{
						m_impl->m_poseConfigs.at(i).m_keyPointIdxsXAxis.emplace_back(item);
					}

					for (const auto& item : jsonFile.at("appConfigs").at("dlcs").at(i).at("poseEstimationConfigs").at("orientationZConfigs").at("keypointIndicesXAxisDoubleCheck"))
					{
						AxisVecIdxs vecIdxs;
						vecIdxs.m_headIdx = item.at("vectorHeadIdx").get<size_t>();
						vecIdxs.m_tailIdx = item.at("vectorTailIdx").get<size_t>();
						m_impl->m_poseConfigs.at(i).m_xAxisDoubleCheckerIdxs.emplace_back(vecIdxs);
					}

					std::string centerpointCalculation = jsonFile.at("appConfigs").at("dlcs").at(i).at("poseEstimationConfigs").at("centerpointConfigs").at("centerpoint");
					if (centerpointCalculation == "fromRect")
					{
						m_impl->m_poseConfigs.at(i).m_centerpointCalcMethod = centerpointCalcMethod::FromRect;
					}
					else if (centerpointCalculation == "fromKeyPoint")
					{
						m_impl->m_poseConfigs.at(i).m_centerpointCalcMethod = centerpointCalcMethod::FromKeyPoints;
					}
					for (int j = 0; j < jsonFile.at("appConfigs").at("dlcs").at(i).at("poseEstimationConfigs").at("centerpointConfigs").at("centerpointIdxs").size(); ++j)
					{
						m_impl->m_poseConfigs.at(i).m_centerpointIdxs.emplace_back(
							jsonFile.at("appConfigs").at("dlcs").at(i).at("poseEstimationConfigs").at("centerpointConfigs").at("centerpointIdxs").at(j).get<int>());
					}

					m_impl->m_poseConfigs.at(i).m_deltaAngleBetweenAxesThresholdRad =
						jsonFile.at("appConfigs").at("dlcs").at(i).at("poseEstimationConfigs").at("orientationZConfigs").at("deltaAngleBetweenAxesThresholdInDegree").get<float>() / 360.0 * 2.0 * CV_PI;

					// Test QualityGateDimension
					m_impl->m_poseConfigs.at(i).m_qualityGateDim.m_enable =
						jsonFile.at("appConfigs").at("dlcs").at(i).at("qualityGateDimension").at("enable");
					for (int j = 0; j < jsonFile.at("appConfigs").at("dlcs").at(i).at("qualityGateDimension").at("idxs").size(); ++j)
					{
						int idx0 = jsonFile.at("appConfigs").at("dlcs").at(i).at("qualityGateDimension").at("idxs").at(j).at("idx0");
						int idx1 = jsonFile.at("appConfigs").at("dlcs").at(i).at("qualityGateDimension").at("idxs").at(j).at("idx1");
						float thr = jsonFile.at("appConfigs").at("dlcs").at(i).at("qualityGateDimension").at("idxs").at(j).at("distanceThrInMeter");
						m_impl->m_poseConfigs.at(i).m_qualityGateDim.m_idxsAndThrs.emplace_back(
							QualityGateDimesion::IdxsAndThr{idx0, idx1, thr});
					}
							}
		}
	}
	catch(const std::exception& e)
	{
		m_impl->m_kLogger->error("Could not read parameters from file: " + networksConfigPath.string());
		return;
	}
	m_moduleConfigured = true;
}

PoseEstimation::PoseEstimation() : m_impl(std::make_unique<Impl>())
{
	;
}


PoseEstimation::~PoseEstimation() {}

float PoseEstimation::Impl::getRotationAngleOfKeyPoints(const std::vector<cv::Point2f>& points,
                                                        const PoseConfig& poseConfig,
														const bool pointsFlipped) const
{
	// configuration does not have a direction vector
	assert(!poseConfig.m_keyPointIdxsXAxis.empty());

	// PCA 2d
	cv::Vec2d directionVec;
	cv::Mat dataPts = cv::Mat(poseConfig.m_keyPointIdxsXAxis.size(), 2, CV_64F);

	// take only the keypoints which are specified in the config file
	for (int i = 0; i < dataPts.rows; ++i)
	{
		dataPts.at<double>(i, 0) = points[poseConfig.m_keyPointIdxsXAxis.at(i)].x;
		dataPts.at<double>(i, 1) = points[poseConfig.m_keyPointIdxsXAxis.at(i)].y;
	}

	// Perform PCA analysis
	cv::PCA pca_analysis(dataPts, cv::Mat(), cv::PCA::DATA_AS_ROW);

	// Store the eigenvalues and eigenvectors
	int nEigenValues = 2;  //(pca_analysis.eigenvalues).size;
	std::vector<double> eigen_val(nEigenValues);
	for (int i = 0; i < 2; i++)
	{
		eigen_val[i] = pca_analysis.eigenvalues.at<double>(i);
	}

	std::vector<double>::iterator it = std::max_element(eigen_val.begin(), eigen_val.end());
	int idxMax = std::distance(eigen_val.begin(), it);

	directionVec[0] = pca_analysis.eigenvectors.at<double>(idxMax, 0);
	directionVec[1] = pca_analysis.eigenvectors.at<double>(idxMax, 1);

	cv::Vec2d keyPointXAxis(
	    points[poseConfig.m_keyPointIdxsXAxis.back()].x - points[poseConfig.m_keyPointIdxsXAxis.at(0)].x,
	    points[poseConfig.m_keyPointIdxsXAxis.back()].y - points[poseConfig.m_keyPointIdxsXAxis.at(0)].y);

    // double check of direction axis with more candidates within the configuration
	// here we are sure that the keypoints are all valid
	std::vector<cv::Vec2d> doubleCheckXAxes;
	for (size_t i = 0; i < poseConfig.m_xAxisDoubleCheckerIdxs.size(); ++i)
	{
		size_t headIdx = poseConfig.m_xAxisDoubleCheckerIdxs.at(i).m_headIdx;
		size_t tailIdx = poseConfig.m_xAxisDoubleCheckerIdxs.at(i).m_tailIdx;
		doubleCheckXAxes.emplace_back(
		    cv::Vec2d(points[headIdx].x - points[tailIdx].x, points[headIdx].y - points[tailIdx].y));
	}

	for (const auto& checkerAxis : doubleCheckXAxes)
	{
		const float angleBetweenAxesRad =
		    std::acos(keyPointXAxis.dot(checkerAxis) / (cv::norm(keyPointXAxis) * cv::norm(checkerAxis)));
		if(poseConfig.m_longShortEdgePoseEstimationEnable && pointsFlipped)
		{
			if (std::abs(angleBetweenAxesRad) > poseConfig.m_deltaAngleBetweenAxesThresholdRad + CV_PI/2.0)
			{
				throw std::runtime_error("two x-axis exceed the maximum acceptable anlge value between each other -> bad "
			 	                        "keypoints or bad config "
			    	                     "file value 'keypointIndicesXAxisDoubleCheck'!");
			}
		}
		else
		{
			if (std::abs(angleBetweenAxesRad) > poseConfig.m_deltaAngleBetweenAxesThresholdRad)
			{
				throw std::runtime_error("two x-axis exceed the maximum acceptable anlge value between each other -> bad "
			 	                        "keypoints or bad config "
			    	                     "file value 'keypointIndicesXAxisDoubleCheck'!");
			}
		}
		
	}

	// corretion of direction vector (aka x-axis)
	// if vectors are pointing in different directions
	// therefore a scalarproduct is computed
	if (keyPointXAxis.dot(directionVec) <= 0)
	{
		directionVec[0] = directionVec[0] * -1;
		directionVec[1] = directionVec[1] * -1;
	}

    // from direction vector to angle
	float radians = atan2(directionVec[1], directionVec[0]);
	radians = radians - CV_PI / 2.0;  // offset of 90°

	return radians;
}

Eigen::Vector3f PoseEstimation::Impl::calcDirectionVector(const VsCameraIntrinsics& intrinsics,
                                                          const std::vector<cv::Point2f>& keyPoints,
                                                          const PoseConfig& poseConfig,
                                                          const pcl::PointXYZ& centerPoint,
                                                          const Eigen::Vector3f& normalVec,
														  const bool pointsFlipped) const
{
	// determining the orientation around the z axis of the camera:
	// giving the keypoints and the aimed configured direction
	float orientationZ = getRotationAngleOfKeyPoints(keyPoints, poseConfig, pointsFlipped);

    // calculating a second point along the direction vector
	constexpr float kMagicDistanceInPixels = 5.0; // length of the direction vector
	cv::Point3f p;
	p.x = centerPoint.x + kMagicDistanceInPixels * std::sin(orientationZ);
	p.y = centerPoint.y - kMagicDistanceInPixels * std::cos(orientationZ);
	p.z = -1 * (normalVec[0] * (p.x - centerPoint.x) + normalVec[1] * (p.y - centerPoint.y)) / normalVec[2]
	      + centerPoint.z;

	// direction vector = P2(x,y,z) - centerPoint(x,y,z)
	Eigen::Vector3f directionVec;
	directionVec[0] = p.x - centerPoint.x;
	directionVec[1] = p.y - centerPoint.y;
	directionVec[2] = p.z - centerPoint.z;

	return directionVec;
}

void PoseEstimation::Impl::getPointcloud(const cv::Mat& mask,
                                         const cv::Mat_<float>& depthImg,
                                         const VsCameraIntrinsics& intrinsics,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr dst,
										 const PoseConfig& poseConfig) const
{
	assert(mask.empty() == false);

    // ignoring zero values, because the depth map is a noisy one, these values make no sense
	std::vector<cv::Point> nonZeroLocationsFromMask;
	cv::findNonZero(mask, nonZeroLocationsFromMask);

    // from depthmap to x,y,z coordinates in the camera frame
	math::makeSensor2CamCoordsCloud(nonZeroLocationsFromMask, dst, intrinsics, depthImg, poseConfig.m_maxDepth, poseConfig.m_minDepth, m_loopOffsetDepthPoints.at(0));
	// apply sor filter on the point cloud to eliminate outliers
	dst = math::applySORFilter(dst);
    // object has < 3 valid points in the point cloud ==> bad depth estimation within the object
	constexpr unsigned int kLowerLimitOfCloudPoints = 3;
	if (dst->points.size() < kLowerLimitOfCloudPoints)
	{
		std::string errorMsg =
		    "Less than " + std::to_string(kLowerLimitOfCloudPoints) + " Points can be used for PoseEstimation";
		throw std::runtime_error(errorMsg);
	}
}

VSPose PoseEstimation::Impl::makeRobotPose(const VSPose& centerOfProductCameraCoords,
                                           const cv::Mat& rotMat,
                                           const cv::Mat& extrinsics)
{
	cv::Mat cam2robot;                             // rotation matrix camera -> robot
	cv::Rect rotPartofMat = cv::Rect(0, 0, 3, 3);  // needed to crop the rotation part (3x3) from extrinsic Matrix (3x4)
	extrinsics(rotPartofMat).convertTo(cam2robot, CV_32FC1);

	cv::Mat product2CameraRot;  // rotation matrix product -> camera
	product2CameraRot = rotMat;
	product2CameraRot.convertTo(product2CameraRot, CV_32FC1);

	cv::Mat_<double> product2RobotTransformation;  // rotation matrix product -> robot
	product2RobotTransformation = cam2robot * product2CameraRot;

	// if zz component is negative then we turn around the z-axis and the y-axis
	// to make the z-axis always pointing downwards. Y-axis is modified to stay in a
	// right handed coordinate system
	if (product2RobotTransformation.at<double>(2, 2) < 0)
	{
		product2RobotTransformation.at<double>(0, 2) = product2RobotTransformation.at<double>(0, 2) * -1.0;
		product2RobotTransformation.at<double>(1, 2) = product2RobotTransformation.at<double>(1, 2) * -1.0;
		product2RobotTransformation.at<double>(2, 2) = product2RobotTransformation.at<double>(2, 2) * -1.0;

		product2RobotTransformation.at<double>(0, 1) = product2RobotTransformation.at<double>(0, 1) * -1.0;
		product2RobotTransformation.at<double>(1, 1) = product2RobotTransformation.at<double>(1, 1) * -1.0;
		product2RobotTransformation.at<double>(2, 1) = product2RobotTransformation.at<double>(2, 1) * -1.0;
	}
	VSPose poseRobot = math::convertCam2RobotCoords(centerOfProductCameraCoords, extrinsics);
	cv::Vec3f vEulerAngles = math::rotationMatrixToEulerAngles(product2RobotTransformation, "ZYX");
	poseRobot.roll = vEulerAngles[0];
	poseRobot.pitch = vEulerAngles[1];
	poseRobot.yaw = vEulerAngles[2];

    // because the robot can not achieve 6D movement by picking and placing at the same time
	// that is why sometimes it is usefull to fix some degrees of freedom
	if (m_fixRollAngle)
	{
		poseRobot.roll = 0.0;
	}
	if (m_fixPitchAngle)
	{
		poseRobot.pitch = 0.0;
	}
	if (m_fixYawAngle)
	{
		poseRobot.yaw = 0.0;
	}

	return poseRobot;
}

void PoseEstimation::Impl::drawPose(const VSPose& poseCameraCoords,
                                    const cv::Mat& product2CameraTransf,
                                    cv::Mat& debugImg,
                                    const VsCameraIntrinsics& intrinsics)
{
	// x,y,z in camera frame ==> u,v in the image
	cv::Point centerPointUV = math::convertCamera2SensorCoords(poseCameraCoords, intrinsics);

	// just for plotting the coordinate system
	// In a rotation matrix each column corresponds to each unit vector after the rotation
	// for example the new unit vector x is the first column
	Eigen::Vector3f xAxis;
	xAxis[0] = product2CameraTransf.at<float>(0, 0);
	xAxis[1] = product2CameraTransf.at<float>(1, 0);
	xAxis[2] = product2CameraTransf.at<float>(2, 0);

	Eigen::Vector3f yAxis;
	yAxis[0] = product2CameraTransf.at<float>(0, 1);
	yAxis[1] = product2CameraTransf.at<float>(1, 1);
	yAxis[2] = product2CameraTransf.at<float>(2, 1);

	Eigen::Vector3f zAxis;
	zAxis[0] = product2CameraTransf.at<float>(0, 2);
	zAxis[1] = product2CameraTransf.at<float>(1, 2);
	zAxis[2] = product2CameraTransf.at<float>(2, 2);

	constexpr float kLengthOfAxis = 0.1;  // SI-Unit, 0.1 meters scaling factor
	// drawing in the image the line showing the x axis
	cv::Point3f pXAxisCKOS = {poseCameraCoords.x + xAxis[0] * kLengthOfAxis,
	                          poseCameraCoords.y + xAxis[1] * kLengthOfAxis,
	                          poseCameraCoords.z + xAxis[2] * kLengthOfAxis};
	cv::Point pXAxis = cv::Point(intrinsics.fx * pXAxisCKOS.x / pXAxisCKOS.z + intrinsics.cx,
	                             intrinsics.fy * pXAxisCKOS.y / pXAxisCKOS.z + intrinsics.cy);
	cv::line(debugImg, centerPointUV, pXAxis, COLOR_RED, 4);

    // drawing in the image the line showing the y axis
	cv::Point3f pYAxisCKOS = {poseCameraCoords.x + yAxis[0] * kLengthOfAxis,
	                          poseCameraCoords.y + yAxis[1] * kLengthOfAxis,
	                          poseCameraCoords.z + yAxis[2] * kLengthOfAxis};
	cv::Point pYAxis = cv::Point(intrinsics.fx * pYAxisCKOS.x / pYAxisCKOS.z + intrinsics.cx,
	                             intrinsics.fy * pYAxisCKOS.y / pYAxisCKOS.z + intrinsics.cy);
	cv::line(debugImg, centerPointUV, pYAxis, COLOR_GREEN, 4);

    // drawing in the image the line showing the z axis
	cv::Point3f pZAxisCKOS = {poseCameraCoords.x + zAxis[0] * kLengthOfAxis,
	                          poseCameraCoords.y + zAxis[1] * kLengthOfAxis,
	                          poseCameraCoords.z + zAxis[2] * kLengthOfAxis};
	cv::Point pZAxis = cv::Point(intrinsics.fx * pZAxisCKOS.x / pZAxisCKOS.z + intrinsics.cx,
	                             intrinsics.fy * pZAxisCKOS.y / pZAxisCKOS.z + intrinsics.cy);
	cv::line(debugImg, centerPointUV, pZAxis, COLOR_BLUE, 4);
}

void PoseEstimation::Impl::planeEstimationCPU(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                              pcl::ModelCoefficients::Ptr modelCoefficientsCPU)
{
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.001);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *modelCoefficientsCPU);

	if (inliers->indices.size() == 0)
	{
		throw std::runtime_error("Could not estimate a planar model for the point cloud.");
	}
}

cv::Mat PoseEstimation::Impl::drawMaskFromKeypoints(const cv::Point centerOfMask,
                                                    const std::vector<cv::Point2f>& keyPoints,
                                                    const cv::Size& imgSize,
                                                    const PoseConfig& poseConfig,
                                                    cv::Mat& debugImg) const
{
	// the calculated mask is the region of interest where a plane should be estimated
	cv::Mat mask = cv::Mat::zeros(imgSize, CV_8UC1);

	// if there is only one keypoint specified in config file we
	// only make a mask in close proximity around that keypoint
	if (poseConfig.m_keyPointIdxsMaskForDepth.size() == 1)
	{
		size_t idxForDepthMask = poseConfig.m_keyPointIdxsMaskForDepth.at(0);
		cv::Point centerOfMask_ = keyPoints.at(idxForDepthMask);
		constexpr int kMagicRadius = 10;
		cv::circle(mask, centerOfMask_, kMagicRadius, cv::Scalar(255), cv::FILLED);
		cv::circle(debugImg, centerOfMask_, kMagicRadius, cv::Scalar(255), cv::LINE_4);
		return mask;
	}
	// if there is no keypoint specified in config file and the center of object is determined using rectangle of the detection
	// only make a mask in close proximity around that center point of the object
	else if (poseConfig.m_keyPointIdxsMaskForDepth.size() == 0)
	{
		constexpr int kMagicRadius = 10;
		cv::circle(mask, centerOfMask, kMagicRadius, cv::Scalar(255), cv::FILLED);
		cv::circle(debugImg, centerOfMask, kMagicRadius, cv::Scalar(255), cv::LINE_4);
		return mask;
	}

	// convert keypoints to int for cv::convexHull
	std::vector<cv::Point> intPoints;

	for (int i = 0; i < poseConfig.m_keyPointIdxsMaskForDepth.size(); ++i)
	{
		size_t idxForDepthMask = poseConfig.m_keyPointIdxsMaskForDepth.at(i);
		intPoints.push_back(keyPoints.at(idxForDepthMask));
	}

	std::vector<cv::Point> hull;
	cv::convexHull(intPoints, hull);

	std::vector<std::vector<cv::Point>> drawInput;
	if (poseConfig.m_useConvexHull) {
        std::vector<cv::Point> hull;
        cv::convexHull(intPoints, hull);
        drawInput.push_back(hull);
        cv::drawContours(mask, drawInput, -1, cv::Scalar(255), cv::FILLED);
		cv::drawContours(debugImg, drawInput, -1, cv::Scalar(255), cv::LINE_4);
    } else {
        drawInput.push_back(intPoints);
        cv::polylines(mask, drawInput, true, cv::Scalar(255), 2);
		cv::polylines(debugImg, drawInput, true, cv::Scalar(255), cv::LINE_4);
    }
    
    
	return mask;
}

cv::Mat_<float> PoseEstimation::Impl::buildRotationMatrix(Eigen::Vector3f& directionVec, Eigen::Vector3f& normalVec)
{
	normalVec = normalVec / normalVec.norm(); // needs to be normed because later rotation matix calculation
	directionVec = directionVec / directionVec.norm();  // needs to be normed because later rotation matix
	                                                    // calculation

	Eigen::Vector3f leftVec = directionVec.cross(normalVec); // left vector is the dXn
	leftVec = leftVec / leftVec.norm();  // needs to be normed because rotation matix calculation

	// build rotation Matrix out of three coordinate axes of a product
	cv::Mat_<float> rotMat = cv::Mat_<float>::zeros(3, 3);
	rotMat.at<float>(0, 0) = directionVec[0];
	rotMat.at<float>(1, 0) = directionVec[1];
	rotMat.at<float>(2, 0) = directionVec[2];
	rotMat.at<float>(0, 1) = -1.0 * leftVec[0];
	rotMat.at<float>(1, 1) = -1.0 * leftVec[1];
	rotMat.at<float>(2, 1) = -1.0 * leftVec[2];
	rotMat.at<float>(0, 2) = normalVec[0];
	rotMat.at<float>(1, 2) = normalVec[1];
	rotMat.at<float>(2, 2) = normalVec[2];
	return rotMat;
}

std::tuple<cv::Mat, cv::Point> PoseEstimation::Impl::getMaskAndcenterpoint2d(const std::vector<cv::Point2f>& keyPoints,
                                                                           const cv::Rect& rectDetection,
                                                                           const cv::Size& imgSize,
                                                                           const PoseConfig& poseConfig,
                                                                           cv::Mat& debugImg) const
{
	// get the center of the mask depending of the methode used
	cv::Point centerOfMask;
	if (poseConfig.m_centerpointCalcMethod == centerpointCalcMethod::FromRect)
	{
		centerOfMask.x = rectDetection.x + rectDetection.width / 2;   // use centerpoint from yolo instead of dlc
		centerOfMask.y = rectDetection.y + rectDetection.height / 2;  // use centerpoint from yolo instead of dlc
	}
	else if (poseConfig.m_centerpointCalcMethod == centerpointCalcMethod::FromKeyPoints)
	{
		centerOfMask = math::getCenterPoint(keyPoints, poseConfig.m_centerpointIdxs);
	}
	// get the mask around this center
	auto maskWithBigContour = drawMaskFromKeypoints(centerOfMask, keyPoints, imgSize, poseConfig, debugImg);

	return {maskWithBigContour, centerOfMask};
}

void PoseEstimation::drawPoseCamCoords(const VSPose& translationCameraCoords,
                                       const cv::Mat& product2CameraRot,
                                       cv::Mat& debugImg,
                                       const VsCameraIntrinsics& intrinsics)
{
	cv::Point centerPointUV = math::convertCamera2SensorCoords(translationCameraCoords, intrinsics);

	// just for plotting the coordinate system
	Eigen::Vector3f xAxis;
	xAxis[0] = product2CameraRot.at<float>(0, 0);
	xAxis[1] = product2CameraRot.at<float>(1, 0);
	xAxis[2] = product2CameraRot.at<float>(2, 0);

	Eigen::Vector3f yAxis;
	yAxis[0] = product2CameraRot.at<float>(0, 1);
	yAxis[1] = product2CameraRot.at<float>(1, 1);
	yAxis[2] = product2CameraRot.at<float>(2, 1);

	Eigen::Vector3f zAxis;
	zAxis[0] = product2CameraRot.at<float>(0, 2);
	zAxis[1] = product2CameraRot.at<float>(1, 2);
	zAxis[2] = product2CameraRot.at<float>(2, 2);

	constexpr float kLengthOfAxis = 0.1;  // SI-Unit, 0.1 meters scaling factor
	cv::Point3f pXAxisCKOS = {translationCameraCoords.x + xAxis[0] * kLengthOfAxis,
	                          translationCameraCoords.y + xAxis[1] * kLengthOfAxis,
	                          translationCameraCoords.z + xAxis[2] * kLengthOfAxis};
	cv::Point pXAxis = cv::Point(intrinsics.fx * pXAxisCKOS.x / pXAxisCKOS.z + intrinsics.cx,
	                             intrinsics.fy * pXAxisCKOS.y / pXAxisCKOS.z + intrinsics.cy);
	cv::line(debugImg, centerPointUV, pXAxis, PoseEstimation::COLOR_RED, 4);

	cv::Point3f pYAxisCKOS = {translationCameraCoords.x + yAxis[0] * kLengthOfAxis,
	                          translationCameraCoords.y + yAxis[1] * kLengthOfAxis,
	                          translationCameraCoords.z + yAxis[2] * kLengthOfAxis};
	cv::Point pYAxis = cv::Point(intrinsics.fx * pYAxisCKOS.x / pYAxisCKOS.z + intrinsics.cx,
	                             intrinsics.fy * pYAxisCKOS.y / pYAxisCKOS.z + intrinsics.cy);
	cv::line(debugImg, centerPointUV, pYAxis, PoseEstimation::COLOR_GREEN, 4);

	cv::Point3f pZAxisCKOS = {translationCameraCoords.x + zAxis[0] * kLengthOfAxis,
	                          translationCameraCoords.y + zAxis[1] * kLengthOfAxis,
	                          translationCameraCoords.z + zAxis[2] * kLengthOfAxis};
	cv::Point pZAxis = cv::Point(intrinsics.fx * pZAxisCKOS.x / pZAxisCKOS.z + intrinsics.cx,
	                             intrinsics.fy * pZAxisCKOS.y / pZAxisCKOS.z + intrinsics.cy);
	cv::line(debugImg, centerPointUV, pZAxis, PoseEstimation::COLOR_BLUE, 4);
}

void PoseEstimation::drawPoseRobotCoords(const VSPose& poseRobotCoords,
                                         const VsCameraIntrinsics& intrinsics,
                                         const cv::Mat& extrinsics,
                                         cv::Mat& dstImg)
{
	// convert from robot to cam coordinates
	cv::Mat poseInCamCoords4x4 = math::convertRobot2CamCoords(poseRobotCoords, extrinsics);
	// call old drawPose
	VSPose translationCamCoords;
	translationCamCoords.x = poseInCamCoords4x4.at<float>(0, 3);
	translationCamCoords.y = poseInCamCoords4x4.at<float>(1, 3);
	translationCamCoords.z = poseInCamCoords4x4.at<float>(2, 3);

	drawPoseCamCoords(translationCamCoords, poseInCamCoords4x4(cv::Rect(0, 0, 3, 3)), dstImg, intrinsics);
}

VSPose PoseEstimation::makeRobotPose(const VSPose& centerOfProductCameraCoords,
                                     const cv::Mat& rotMat,
                                     const cv::Mat& extrinsics)
{
	cv::Mat cam2robot;                             // rotation matrix camera -> robot
	cv::Rect rotPartofMat = cv::Rect(0, 0, 3, 3);  // needed to crop the rotation part (3x3) from extrinsic Matrix (3x4)
	extrinsics(rotPartofMat).convertTo(cam2robot, CV_32FC1);

	cv::Mat product2CameraRot;  // rotation matrix product -> camera
	product2CameraRot = rotMat;
	product2CameraRot.convertTo(product2CameraRot, CV_32FC1);

	cv::Mat_<double> product2RobotTransformation;  // rotation matrix product -> robot
	product2RobotTransformation = cam2robot * product2CameraRot;

	// if zz component is negative then we turn around the z-axis and the y-axis
	// to make the z-axis always pointing downwards. Y-axis is modified to stay in a
	// right handed coordinate system
	if (product2RobotTransformation.at<double>(2, 2) < 0)
	{
		product2RobotTransformation.at<double>(0, 2) = product2RobotTransformation.at<double>(0, 2) * -1.0;
		product2RobotTransformation.at<double>(1, 2) = product2RobotTransformation.at<double>(1, 2) * -1.0;
		product2RobotTransformation.at<double>(2, 2) = product2RobotTransformation.at<double>(2, 2) * -1.0;

		product2RobotTransformation.at<double>(0, 1) = product2RobotTransformation.at<double>(0, 1) * -1.0;
		product2RobotTransformation.at<double>(1, 1) = product2RobotTransformation.at<double>(1, 1) * -1.0;
		product2RobotTransformation.at<double>(2, 1) = product2RobotTransformation.at<double>(2, 1) * -1.0;
	}
	VSPose poseRobot = math::convertCam2RobotCoords(centerOfProductCameraCoords, extrinsics);
	cv::Vec3f vEulerAngles = math::rotationMatrixToEulerAngles(product2RobotTransformation, "ZYX");
	poseRobot.roll = vEulerAngles[0];
	poseRobot.pitch = vEulerAngles[1];
	poseRobot.yaw = vEulerAngles[2];

	// m_kLogger->debug("convertCam2RobotCoords() - poseRobot.x = {}", poseRobot.x);
	// m_kLogger->debug("convertCam2RobotCoords() - poseRobot.y = {}", poseRobot.y);
	// m_kLogger->debug("convertCam2RobotCoords() - poseRobot.z = {}", poseRobot.z);
	// m_kLogger->debug("convertCam2RobotCoords() - poseRobot.roll = {}", poseRobot.roll);
	// m_kLogger->debug("convertCam2RobotCoords() - poseRobot.pitch = {}", poseRobot.pitch);
	// m_kLogger->debug("convertCam2RobotCoords() - poseRobot.yaw = {}", poseRobot.yaw);

	return poseRobot;
}

void PoseEstimation::Impl::calccenterpoint3d(const cv::Mat_<float>& depthImg,
                                           const float constZInRobotCoords,
                                           const cv::Mat& maskWithBigContour,
                                           const cv::Point& centerOfMask,
                                           const VsCameraIntrinsics& intrinsics,
                                           const cv::Mat_<double>& extrinsics,
                                           pcl::PointXYZ& centerpoint3d,
                                           Eigen::Vector3f& normalVec,
										   const PoseConfig& poseConfig)
{
	if (m_constDepthEnable)
	{
		float factorX = (centerOfMask.x - intrinsics.cx) / intrinsics.fx;
		float factorY = (centerOfMask.y - intrinsics.cy) / intrinsics.fy;
		float offsetZ = extrinsics.at<double>(2, 3);
		float zInCameraCoords = (constZInRobotCoords - offsetZ)
		                        / (extrinsics.at<double>(2, 0) * factorX + extrinsics.at<double>(2, 1) * factorY
		                           + extrinsics.at<double>(2, 2));
		centerpoint3d = math::convertSensor2CamCoords(intrinsics, centerOfMask, zInCameraCoords);

		cv::Mat_<double> normalVecRobotCoords = (cv::Mat_<double>(3, 1) << 0, 0, 1);
		cv::Mat normal = extrinsics(cv::Rect(0, 0, 3, 3)).t() * normalVecRobotCoords;
		cv::cv2eigen(normal, normalVec);
	}
	else
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
		getPointcloud(maskWithBigContour, depthImg, intrinsics, cloud, poseConfig);

		// modelCoefficients are a,b,c and d in equation:
		// a*x + b*y + c*z = d
		pcl::ModelCoefficients::Ptr modelCoefficients(new pcl::ModelCoefficients());
		planeEstimationCPU(cloud, modelCoefficients);

		pcl::PointXYZ centerPointCloud = math::calcCenterOfPointCloud(cloud);

		// recalculate parameter d of plane equation because
		// cudaSegmentation() gives sometimes nonsensical values for d
		// d = a*x+ b*y + c*z
		modelCoefficients->values[3] = modelCoefficients->values[0] * centerPointCloud.x
		                               + modelCoefficients->values[1] * centerPointCloud.y
		                               + modelCoefficients->values[2] * centerPointCloud.z;

		normalVec << modelCoefficients->values[0], modelCoefficients->values[1], modelCoefficients->values[2];

		centerpoint3d =
		    math::makeSensor2CamCoordsWithPlane(depthImg, intrinsics, normalVec, centerOfMask, modelCoefficients);
	}
}

void PoseEstimation::Impl::calcXAxisWithLongShortEdge(const VsCameraIntrinsics& intrinsics,
													  const cv::Mat_<double>& extrinsics,
                                                      const cv::Mat_<float>& depthImg,
                                                      const int classId,
                                                      std::vector<cv::Point2f>& keyPoints,
                                                      PoseConfig& poseConfig,
                                                      cv::Mat& debugImg,
													  bool& pointsFlipped)
{
	assert(!depthImg.empty());
	assert(!keyPoints.empty());

	float distance01 = 0.0;
	if (m_constDepthEnable)
	{
		float factorX0 = (keyPoints.at(poseConfig.m_idxOfKeyPointForAxis0).x - intrinsics.cx)
			                 / intrinsics.fx;
		float factorY0 = (keyPoints.at(poseConfig.m_idxOfKeyPointForAxis0).y - intrinsics.cy)
			                 / intrinsics.fy;
		float offsetZ0 = extrinsics.at<double>(2, 3);
		float zInCameraCoords0 = (m_classToValuesInMeterMapping.at(classId) - offsetZ0)
			                         / (extrinsics.at<double>(2, 0) * factorX0 + extrinsics.at<double>(2, 1) * factorY0
			                            + extrinsics.at<double>(2, 2));

		float factorX1 = (keyPoints.at(poseConfig.m_idxOfKeyPointForAxis1).x - intrinsics.cx)
			                 / intrinsics.fx;
		float factorY1 = (keyPoints.at(poseConfig.m_idxOfKeyPointForAxis1).y - intrinsics.cy)
			                 / intrinsics.fy;
		float offsetZ1 = extrinsics.at<double>(2, 3);
		float zInCameraCoords1 = (m_classToValuesInMeterMapping.at(classId) - offsetZ1)
			                         / (extrinsics.at<double>(2, 0) * factorX1 + extrinsics.at<double>(2, 1) * factorY1
			                            + extrinsics.at<double>(2, 2));
		distance01 = math::calcEuclidianDistanceCamCoords(keyPoints.at(poseConfig.m_idxOfKeyPointForAxis0),
		                                                  zInCameraCoords0,
		                                                  keyPoints.at(poseConfig.m_idxOfKeyPointForAxis1),
		                                                  zInCameraCoords1,
		                                                  intrinsics);

		std::cout<<"Distance Long Short "<<distance01<<std::endl;
	}
	else
	{
		distance01 =
		    math::calcEuclidianDistanceCamCoords(keyPoints.at(poseConfig.m_idxOfKeyPointForAxis0),
		                                         depthImg.at<float>(keyPoints.at(poseConfig.m_idxOfKeyPointForAxis0)),
		                                         keyPoints.at(poseConfig.m_idxOfKeyPointForAxis1),
		                                         depthImg.at<float>(keyPoints.at(poseConfig.m_idxOfKeyPointForAxis1)),
		                                         intrinsics);
	}

	cv::Point2f xAxisP0 = keyPoints.at(poseConfig.m_idxOfKeyPointForAxis0);
	cv::Point2f xAxisP1 = keyPoints.at(poseConfig.m_idxOfKeyPointForAxis1);
	// m_kLogger->debug("distance01 {}", distance01);
	if (distance01 < poseConfig.m_longShortEdgeLengthInMeterThr)  // in this case we found the y axis
	{
		cv::Point2f yAxisVec = xAxisP1 - xAxisP0;
		cv::Point2f yAxisVecNorm = yAxisVec / std::sqrt(yAxisVec.x * yAxisVec.x + yAxisVec.y * yAxisVec.y);
		float tmpCoord = yAxisVecNorm.x;
		yAxisVecNorm.x = yAxisVecNorm.y * -1.0;  // make perpendicular vec (make y to x axis)
		yAxisVecNorm.y = tmpCoord;

		constexpr float kMagicVectorLengthInPxls = 40.0;
		xAxisP1 = yAxisVecNorm * kMagicVectorLengthInPxls + xAxisP0;
		keyPoints.emplace_back(xAxisP1);  // create artificial additional point to create a x axis
		poseConfig.m_idxOfKeyPointForAxis1 = keyPoints.size() - 1;
		pointsFlipped = true;
	}
	std::vector<cv::Point2f> xAxisPoints = {xAxisP0, xAxisP1};

	const int dlcIdx = m_classToDlcIdxMapping.at(classId);

	poseConfig.m_keyPointIdxsMaskForDepth = m_poseConfigs.at(dlcIdx).m_keyPointIdxsMaskForDepth;
	poseConfig.m_keyPointIdxsXAxis = std::vector<unsigned int>(xAxisPoints.size());
	poseConfig.m_centerpointCalcMethod = m_poseConfigs.at(dlcIdx).m_centerpointCalcMethod;
	poseConfig.m_keyPointIdxsXAxis.at(0) = poseConfig.m_idxOfKeyPointForAxis0;
	poseConfig.m_keyPointIdxsXAxis.at(1) = poseConfig.m_idxOfKeyPointForAxis1;
	poseConfig.m_centerpointIdxs = m_poseConfigs.at(dlcIdx).m_centerpointIdxs;

}

VSPose PoseEstimation::Impl::calcPoseWithKeyPoints(const VsCameraIntrinsics& intrinsics,
                                                   const cv::Mat_<double>& extrinsics,
                                                   const cv::Mat_<float>& depthImg,
                                                   const float constZInRobotCoords,
                                                   const std::vector<cv::Point2f>& keyPoints,
                                                   const PoseConfig& poseConfig,
                                                   const cv::Rect& rectDetection,
                                                   const int classId,
                                                   cv::Mat& debugImg,
												   const bool pointsFlipped)
{
	auto [maskWithBigContour, centerOfMask] =
	    getMaskAndcenterpoint2d(keyPoints, rectDetection, depthImg.size(), poseConfig, debugImg);

	pcl::PointXYZ centerpoint3d;
	Eigen::Vector3f normalVec;
	calccenterpoint3d(depthImg,
	                constZInRobotCoords,
	                maskWithBigContour,
	                centerOfMask,
	                intrinsics,
	                extrinsics,
	                centerpoint3d,
	                normalVec,
					poseConfig);

	Eigen::Vector3f directionVec = calcDirectionVector(intrinsics, keyPoints, poseConfig, centerpoint3d, normalVec, pointsFlipped);

	cv::Mat rotMat = buildRotationMatrix(directionVec, normalVec);

	VSPose centerOfProductCameraCoords;
	centerOfProductCameraCoords.x = centerpoint3d.x;
	centerOfProductCameraCoords.y = centerpoint3d.y;
	centerOfProductCameraCoords.z = centerpoint3d.z;
	drawPose(centerOfProductCameraCoords, rotMat, debugImg, intrinsics);
	VSPose poseInRobotCoords = makeRobotPose(centerOfProductCameraCoords, rotMat, extrinsics);
	if(poseConfig.m_layersConfig.m_useLayers)
	{
		for (int j = 0; j < poseConfig.m_layersConfig.m_layersValues.size(); ++j)
		{
			if (poseInRobotCoords.z <= poseConfig.m_layersConfig.m_layersValues.at(j) + poseConfig.m_layersConfig.m_layersResolution &&
			poseInRobotCoords.z >= poseConfig.m_layersConfig.m_layersValues.at(j) - poseConfig.m_layersConfig.m_layersResolution)
			{
				poseInRobotCoords.z = poseConfig.m_layersConfig.m_layersValues.at(j);
				return poseInRobotCoords;
			}
		}
	}
	return poseInRobotCoords;
}

bool PoseEstimation::Impl::checkDimQualityGate(const VsCameraIntrinsics& intrinsics,
                                               const cv::Mat_<double>& extrinsics,
                                               const cv::Mat_<float>& depthImg,
                                               const int classId,
                                               const std::vector<cv::Point2f>& keyPoints,
                                               PoseConfig& poseConfig,
                                               cv::Mat& debugImg)
{
	assert(!keyPoints.empty());
	// Calculate the dimensions of object
	std::vector<float> distances;
	// For every set of two keypoints
	for (int i = 0; i < poseConfig.m_qualityGateDim.m_idxsAndThrs.size(); ++i)
	{
		// calculate the distance between those keypoints
		if (m_constDepthEnable)
		{
			float factorX0 = (keyPoints.at(poseConfig.m_qualityGateDim.m_idxsAndThrs.at(i).m_idx0).x - intrinsics.cx)
			                 / intrinsics.fx;
			float factorY0 = (keyPoints.at(poseConfig.m_qualityGateDim.m_idxsAndThrs.at(i).m_idx0).y - intrinsics.cy)
			                 / intrinsics.fy;
			float offsetZ0 = extrinsics.at<double>(2, 3);
			float zInCameraCoords0 = (m_classToValuesInMeterMapping.at(classId) - offsetZ0)
			                         / (extrinsics.at<double>(2, 0) * factorX0 + extrinsics.at<double>(2, 1) * factorY0
			                            + extrinsics.at<double>(2, 2));

			float factorX1 = (keyPoints.at(poseConfig.m_qualityGateDim.m_idxsAndThrs.at(i).m_idx1).x - intrinsics.cx)
			                 / intrinsics.fx;
			float factorY1 = (keyPoints.at(poseConfig.m_qualityGateDim.m_idxsAndThrs.at(i).m_idx1).y - intrinsics.cy)
			                 / intrinsics.fy;
			float offsetZ1 = extrinsics.at<double>(2, 3);
			float zInCameraCoords1 = (m_classToValuesInMeterMapping.at(classId) - offsetZ1)
			                         / (extrinsics.at<double>(2, 0) * factorX1 + extrinsics.at<double>(2, 1) * factorY1
			                            + extrinsics.at<double>(2, 2));

			distances.emplace_back(math::calcEuclidianDistanceCamCoords(
			    keyPoints.at(poseConfig.m_qualityGateDim.m_idxsAndThrs.at(i).m_idx0),
			    zInCameraCoords0,
			    keyPoints.at(poseConfig.m_qualityGateDim.m_idxsAndThrs.at(i).m_idx1),
			    zInCameraCoords1,
			    intrinsics));
		}
		else
		{
			distances.emplace_back(math::calcEuclidianDistanceCamCoords(
			    keyPoints.at(poseConfig.m_qualityGateDim.m_idxsAndThrs.at(i).m_idx0),
			    depthImg.at<float>(keyPoints.at(poseConfig.m_qualityGateDim.m_idxsAndThrs.at(i).m_idx0)),
			    keyPoints.at(poseConfig.m_qualityGateDim.m_idxsAndThrs.at(i).m_idx1),
			    depthImg.at<float>(keyPoints.at(poseConfig.m_qualityGateDim.m_idxsAndThrs.at(i).m_idx1)),
			    intrinsics));
		}
		// distance is less than expected ==> bad object e.g. bombage
		if (distances.at(i) < poseConfig.m_qualityGateDim.m_idxsAndThrs.at(i).m_distanceThr)
		{
			return false; 
		}
	}
	return true;
}

VSPose PoseEstimation::getPose(const VsCameraIntrinsics& intrinsics,
                               const cv::Mat_<double>& extrinsics,
                               const cv::Mat_<float>& depthImg,
                               std::vector<cv::Point2f>& keyPoints,
                               const cv::Rect& rectDetection,
                               int& classId,
                               cv::Mat& debugImg)
{
	// inputs are empty ==> error
	assert(!depthImg.empty());
	assert(!keyPoints.empty());
	assert(!extrinsics.empty());

	const int dlcIdx = m_impl->m_classToDlcIdxMapping.at(classId);
	PoseConfig poseConfig = m_impl->m_poseConfigs.at(dlcIdx);
    
	// calculate the 6D pose
	bool pointsFlipped = false; //This is needed for long short edges, if we flip the points
	if (poseConfig.m_longShortEdgePoseEstimationEnable)
	{
		m_impl->calcXAxisWithLongShortEdge(intrinsics,extrinsics, depthImg, classId, keyPoints, poseConfig, debugImg, pointsFlipped);
	}
	VSPose poseInRobotCoords = m_impl->calcPoseWithKeyPoints(intrinsics,
	                                                         extrinsics,
	                                                         depthImg,
	                                                         m_impl->m_classToValuesInMeterMapping.at(classId),
	                                                         keyPoints,
	                                                         poseConfig,
	                                                         rectDetection,
	                                                         classId,
	                                                         debugImg,
															 pointsFlipped);
	// quality gate for dimensions for example bombage products
	if (poseConfig.m_qualityGateDim.m_enable)
	{
		bool objectOk =
		    m_impl->checkDimQualityGate(intrinsics, extrinsics, depthImg, classId, keyPoints, poseConfig, debugImg);
		if (!objectOk)
		{
			classId = CLASS_BAD_OBJECT_DIM;  // class for bad object
		}
	}

	return poseInRobotCoords;
}

VSPose PoseEstimation::getPoseTray(const VsCameraIntrinsics& intrinsics,
                                   const cv::Mat_<double>& extrinsics,
                                   const cv::Rect& rect,
                                   const float zInRobotCoords)
{
	double centerU = rect.x + rect.width / 2;
	double centerV = rect.y + rect.height / 2;
	cv::Point rectCenter(centerU, centerV);
	float factorX = (rectCenter.x - intrinsics.cx) / intrinsics.fx;
	float factorY = (rectCenter.y - intrinsics.cy) / intrinsics.fy;
	float offsetZ = extrinsics.at<double>(2, 3);
	float zInCameraCoords =
	    (zInRobotCoords - offsetZ)
	    / (extrinsics.at<double>(2, 0) * factorX + extrinsics.at<double>(2, 1) * factorY + extrinsics.at<double>(2, 2));
	VSPose pose = math::convertSensor2RobotCoords(intrinsics, extrinsics, rectCenter, zInCameraCoords);
	pose.z = zInRobotCoords;
	return pose;
}
