/**
 * @copyright Copyright (c) 2020 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * @brief mathematical helper functions
 */

#ifndef MATH_HELPER_FUNCS_H
#define MATH_HELPER_FUNCS_H

#include <pcl/ModelCoefficients.h>
#include <pcl/common/distances.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <cameraInterface.hpp>
#include <opencv2/opencv.hpp>

#include "vs_poseObject.h"

namespace math
{
cv::Vec3f rotationMatrixToEulerAngles(const cv::Mat_<double>& R, const std::string& seq);

/**
 * @brief converts sensor coordinates to camera coodinates AND converts from [m] to [mm]
 *
 * @param src vector of sensor coordinates which should be converted to xyz camera coordinates.
 *            z is already known from depthImg
 * @param dst
 * @param intrinsics
 * @param depthImg should contain depth informations in [m]
 */
void makeSensor2CamCoordsCloud(const std::vector<cv::Point>& src,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr dst,
                               const VsCameraIntrinsics& intrinsics,
                               const cv::Mat_<float>& depthImg,
                               const float maxDepth,
                               const float minDepth,
                               const int loopOffsetDepthPoints);

/**
 * @brief calculate the camera coordinates with help of a plane equation (to get a z value)
 *
 * @param intrinsics
 * @param normalVec
 * @param point
 * @param modelCoefficients
 * @return pcl::PointXYZ
 */
pcl::PointXYZ makeSensor2CamCoordsWithPlane(const cv::Mat_<float>& depthImg,
                                            const VsCameraIntrinsics& intrinsics,
                                            const Eigen::Vector3f& normalVec,
                                            const cv::Point& point,
                                            const pcl::ModelCoefficients::Ptr modelCoefficients);

/**
 * @brief converts one point from sensor coordinates in camera coordinates
 *
 * @param intrinsics
 * @param point
 * @param z the depth [mm] of the point has to be known in order to calc the XYZ
 * @return pcl::PointXYZ
 */
pcl::PointXYZ convertSensor2CamCoords(const VsCameraIntrinsics& intrinsics, const cv::Point& point, float z);

/**
 * @brief converts camera to sensor coordinates
 *
 * @param pose
 * @param intrinsics
 * @return cv::Point
 */
cv::Point convertCamera2SensorCoords(const VSPose& pose, const VsCameraIntrinsics& intrinsics);

cv::Point convertCamera2SensorCoords(const pcl::PointXYZ& point, const VsCameraIntrinsics& intrinsics);

/**
 * @brief  converts a pose from camera coordinates to robot coordinates. but only the transitional
 *  componets of the pose. It never touches the roll pitch and yaw components.
 *
 * @param pose pose in camera coordinates
 * @param extrinsics
 * @return VSPose pose in robot coordinates
 */
VSPose convertCam2RobotCoords(const VSPose& pose, const cv::Mat_<double>& extrinsics);

VSPose convertSensor2RobotCoords(const VsCameraIntrinsics& intrinsics,
                                 const cv::Mat_<double>& extrinsics,
                                 const cv::Point& point,
                                 float zInCameraCoords);

cv::Mat convertRobot2CamCoords(const VSPose& pose, const cv::Mat_<double>& extrinsics);

cv::Mat convertPoseEuler2Mat(const VSPose& pose);

float calcEuclidianDistanceCamCoords(const cv::Point& point0,
                                     const float depth0CamCooords,
                                     const cv::Point& point1,
                                     const float depth1CamCooords,
                                     const VsCameraIntrinsics& intrinsics);

pcl::PointXYZ calcCenterOfPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

cv::Point getCenterPoint(const std::vector<cv::Point2f>& points, const std::vector<int>& pointIdxs);

pcl::PointCloud<pcl::PointXYZ>::Ptr applySORFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
}  // namespace math

#endif
