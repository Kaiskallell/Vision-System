
/**
 * @copyright Copyright (c) 2020 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 */

#ifndef POSE_ESTIMATION_H
#define POSE_ESTIMATION_H

#include <experimental/filesystem>
#include <opencv2/opencv.hpp>

#include "cameraInterface.hpp"
#include "logging.h"
#include "vs_poseObject.h"

//! @param define default class for bad object after checking the dimension quality of an object: product or tray
#define CLASS_BAD_OBJECT_DIM 8

namespace fs = std::experimental::filesystem;

struct AxisVecIdxs
{
	size_t m_headIdx = 0;
	size_t m_tailIdx = 0;
};

enum class centerpointCalcMethod
{
	FromKeyPoints = 0,
	FromRect = 1,
	FromSingleKeyPoint = 2,
	fromLineKeyPoints = 3
};

struct QualityGateDimesion
{
	struct IdxsAndThr
	{
		IdxsAndThr(int idx0, int idx1, float thr) : m_idx0(idx0), m_idx1(idx1), m_distanceThr(thr) {}
		int m_idx0 = 0;
		int m_idx1 = 0;
		float m_distanceThr = 0.0;
	};
	bool m_enable = false;
	std::vector<IdxsAndThr> m_idxsAndThrs = {};
};

struct Layers
{
	bool m_useLayers = false;
	std::vector<float> m_layersValues = {};
	float m_layersResolution = 0.0;

};

struct PoseConfig
{
	std::vector<unsigned int> m_keyPointIdxsMaskForDepth = {};
	std::vector<unsigned int> m_keyPointIdxsXAxis = {};
	std::vector<AxisVecIdxs> m_xAxisDoubleCheckerIdxs = {};
	float m_deltaAngleBetweenAxesThresholdRad = 0.0;
	centerpointCalcMethod m_centerpointCalcMethod;
	std::vector<int> m_centerpointIdxs = {};
	bool m_longShortEdgePoseEstimationEnable = false;
	float m_longShortEdgeLengthInMeterThr = 0.05;
	size_t m_idxOfKeyPointForAxis0 = 0;
	size_t m_idxOfKeyPointForAxis1 = 1;
	QualityGateDimesion m_qualityGateDim;
	bool m_useConvexHull = false;
	float m_maxDepth = 1.5;
	float m_minDepth = 0.3;
	Layers m_layersConfig;
};

class PoseEstimation
{
  public:
	PoseEstimation(const fs::path& networksConfigPath);
	PoseEstimation();
	~PoseEstimation();

	VSPose getPose(const VsCameraIntrinsics& intrinsics,
	               const cv::Mat_<double>& m_extrinsics,
	               const cv::Mat_<float>& depthImg,
	               std::vector<cv::Point2f>& keyPoints,
	               const cv::Rect& rectDetection,
	               int& classId,
	               cv::Mat& debugImg);
	VSPose getPoseTray(const VsCameraIntrinsics& intrinsics,
	                   const cv::Mat_<double>& extrinsics,
	                   const cv::Rect& rect,
	                   const float zInRobotCoords);

	inline static const cv::Scalar COLOR_RED = cv::Scalar(0, 0, 255);
	inline static const cv::Scalar COLOR_BLUE = cv::Scalar(255, 0, 0);
	inline static const cv::Scalar COLOR_GREEN = cv::Scalar(0, 128, 0);
	static void drawPoseCamCoords(const VSPose& translationCameraCoords,
	                              const cv::Mat& product2CameraRot,
	                              cv::Mat& debugImg,
	                              const VsCameraIntrinsics& intrinsics);
	static void drawPoseRobotCoords(const VSPose& poseRobotCoords,
	                                const VsCameraIntrinsics& intrinsics,
	                                const cv::Mat& extrinsics,
	                                cv::Mat& dstImg);
	static VSPose makeRobotPose(const VSPose& centerOfProductCameraCoords,
	                            const cv::Mat& rotMat,
	                            const cv::Mat& extrinsics);

	bool m_moduleConfigured = false;

  private:
	class Impl;
	std::unique_ptr<Impl> m_impl;
};

#endif
