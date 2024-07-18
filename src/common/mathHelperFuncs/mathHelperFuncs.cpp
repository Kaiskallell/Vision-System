/**
 * @copyright Copyright (c) 2020 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 */

#include "mathHelperFuncs.h"

namespace math
{
// Calculates rotation matrix to euler angles XYZ
cv::Vec3f rotationMatrixToEulerAngles(const cv::Mat_<double>& R, const std::string& seq)
{
	float x, y, z;
	if (seq == "XYZ")  // TODO(aschaefer): this was never used --> remove it
	{
		float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
		bool bSingular = sy < 1e-6;  // If

		if (!bSingular)
		{
			x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
			y = atan2(-R.at<double>(2, 0), sy);
			z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
		}
		else
		{
			x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
			y = atan2(-R.at<double>(2, 0), sy);
			z = 0;
		}
	}

	if (seq == "ZYX")
	{
		if (R.at<double>(2, 0) != 1 && R.at<double>(2, 0) != -1)
		{
			y = asin(-1 * R.at<double>(2, 0));
			x = atan2(R.at<double>(2, 1) / cos(y), R.at<double>(2, 2) / cos(y));
			z = atan2(R.at<double>(1, 0) / cos(y), R.at<double>(0, 0) / cos(y));
		}
		else
		{
			if (R.at<double>(2, 0) == 1)
			{
				y = -M_PI / 2;
				z = 0;
				x = -z + atan2(-R.at<double>(0, 1), -R.at<double>(0, 2));
			}
			if (R.at<double>(2, 0) == -1)
			{
				y = M_PI / 2;
				z = 0;
				x = -z + atan2(R.at<double>(0, 1), R.at<double>(0, 2));
			}
		}
	}
	return cv::Vec3f(x, y, z);
}

void makeSensor2CamCoordsCloud(const std::vector<cv::Point>& src,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr dst,
                               const VsCameraIntrinsics& intrinsics,
                               const cv::Mat_<float>& depthImg,
                               const float maxDepth,
                               const float minDepth,
                               const int loopOffsetDepthPoints)
{
	float z = 0;
	float x = 0;
	float y = 0;
	for (int i = 0; i < src.size(); i += loopOffsetDepthPoints)
	{
		z = depthImg.at<float>(src[i].y, src[i].x);

		// check if depth info is unrealistically close or far away
		if (z > minDepth && z < maxDepth &&!std::isinf(z))
		{
			dst->push_back(convertSensor2CamCoords(intrinsics, src[i], z));
		}
	}
}

pcl::PointXYZ makeSensor2CamCoordsWithPlane(const cv::Mat_<float>& depthImg,
                                            const VsCameraIntrinsics& intrinsics,
                                            const Eigen::Vector3f& normalVec,
                                            const cv::Point& point,
                                            const pcl::ModelCoefficients::Ptr modelCoefficients)
{
	double q1 = normalVec[0] / intrinsics.fx;
	double q2 = normalVec[1] / intrinsics.fy;

	// double z = depthImg.at<float>(point.y, point.x);

	// Formula obtained by solving the equation of intrinsic Camera Parameter for camera coordinates
	// and solving the plane equation a*x+ b*y + c*z = d
	double z =
	   modelCoefficients->values[3] / (q1 * (point.x - intrinsics.cx) + q2 * (point.y - intrinsics.cy) +
	   normalVec[2]);
	return convertSensor2CamCoords(intrinsics, point, z);
}

pcl::PointXYZ convertSensor2CamCoords(const VsCameraIntrinsics& intrinsics, const cv::Point& point, float z)
{
	cv::Vec<cv::Point2f, 1> points(point);
	// Build the camera matrix
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64FC1);
	cameraMatrix.at<double>(0, 0) = intrinsics.fx;
	cameraMatrix.at<double>(1, 1) = intrinsics.fy;
	cameraMatrix.at<double>(0, 2) = intrinsics.cx;
	cameraMatrix.at<double>(1, 2) = intrinsics.cy;
	cv::Matx33f cameraMatrixMatx(cameraMatrix);

	// Build the distortion coefficients
	cv::Mat distortionCoefficients = cv::Mat::zeros(1, 5, CV_64FC1);
	distortionCoefficients.at<double>(0, 0) = intrinsics.k1;
	distortionCoefficients.at<double>(0, 1) = intrinsics.k2;
	distortionCoefficients.at<double>(0, 2) = intrinsics.p1;
	distortionCoefficients.at<double>(0, 3) = intrinsics.p2;
	distortionCoefficients.at<double>(0, 4) = intrinsics.k3;
	cv::Vec<float, 5> distortionCoefficientsVec(distortionCoefficients);

	// undistored Point
	cv::Vec<cv::Point2f, 1> undistortedPoint;
	cv::undistortPoints(
	    points, undistortedPoint, cameraMatrixMatx, distortionCoefficientsVec, cv::noArray(), cameraMatrixMatx);
	float x = static_cast<float>((z / intrinsics.fx) * (static_cast<float>(undistortedPoint[0].x) - intrinsics.cx));
	float y = static_cast<float>((z / intrinsics.fy) * (static_cast<float>(undistortedPoint[0].y) - intrinsics.cy));

	return pcl::PointXYZ(x, y, z);
}

cv::Point convertCamera2SensorCoords(const VSPose& pose, const VsCameraIntrinsics& intrinsics)
{
	cv::Point2f point(intrinsics.fx * pose.x / pose.z + intrinsics.cx, intrinsics.fy * pose.y / pose.z + intrinsics.cy);
	cv::Vec<cv::Point2f, 1> points(point);
	// Build the camera matrix
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64FC1);
	cameraMatrix.at<double>(0, 0) = intrinsics.fx;
	cameraMatrix.at<double>(1, 1) = intrinsics.fy;
	cameraMatrix.at<double>(0, 2) = intrinsics.cx;
	cameraMatrix.at<double>(1, 2) = intrinsics.cy;
	cv::Matx33f cameraMatrixMatx(cameraMatrix);

	// Build the distortion coefficients
	cv::Mat distortionCoefficients = cv::Mat::zeros(1, 5, CV_64FC1);
	distortionCoefficients.at<double>(0, 0) = -intrinsics.k1;
	distortionCoefficients.at<double>(0, 1) = -intrinsics.k2;
	distortionCoefficients.at<double>(0, 2) = -intrinsics.p1;
	distortionCoefficients.at<double>(0, 3) = -intrinsics.p2;
	distortionCoefficients.at<double>(0, 4) = -intrinsics.k3;
	cv::Vec<float, 5> distortionCoefficientsVec(distortionCoefficients);

	// undistored Point
	cv::Vec<cv::Point2f, 1> undistortedPoint;
	cv::undistortPoints(
	    points, undistortedPoint, cameraMatrixMatx, distortionCoefficientsVec, cv::noArray(), cameraMatrixMatx);
	return undistortedPoint[0];
}

cv::Point convertCamera2SensorCoords(const pcl::PointXYZ& point, const VsCameraIntrinsics& intrinsics)
{
	cv::Point2f point_(intrinsics.fx * point.x / point.z + intrinsics.cx,
	                   intrinsics.fy * point.y / point.z + intrinsics.cy);
	cv::Vec<cv::Point2f, 1> points(point_);
	// Build the camera matrix
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64FC1);
	cameraMatrix.at<double>(0, 0) = intrinsics.fx;
	cameraMatrix.at<double>(1, 1) = intrinsics.fy;
	cameraMatrix.at<double>(0, 2) = intrinsics.cx;
	cameraMatrix.at<double>(1, 2) = intrinsics.cy;
	cv::Matx33f cameraMatrixMatx(cameraMatrix);

	// Build the distortion coefficients
	cv::Mat distortionCoefficients = cv::Mat::zeros(1, 5, CV_64FC1);
	distortionCoefficients.at<double>(0, 0) = -intrinsics.k1;
	distortionCoefficients.at<double>(0, 1) = -intrinsics.k2;
	distortionCoefficients.at<double>(0, 2) = -intrinsics.p1;
	distortionCoefficients.at<double>(0, 3) = -intrinsics.p2;
	distortionCoefficients.at<double>(0, 4) = -intrinsics.k3;
	cv::Vec<float, 5> distortionCoefficientsVec(distortionCoefficients);

	// undistored Point
	cv::Vec<cv::Point2f, 1> undistortedPoint;
	cv::undistortPoints(
	    points, undistortedPoint, cameraMatrixMatx, distortionCoefficientsVec, cv::noArray(), cameraMatrixMatx);
	return undistortedPoint[0];
}

// this only transforms x,y,z but not roll pitch and yaw
VSPose convertCam2RobotCoords(const VSPose& pose, const cv::Mat_<double>& extrinsics)
{
	VSPose poseRobot;  // pose in robot coordinates
	poseRobot.x = pose.x * extrinsics.at<double>(0, 0) + pose.y * extrinsics.at<double>(0, 1)
	              + pose.z * extrinsics.at<double>(0, 2) + extrinsics.at<double>(0, 3);
	poseRobot.y = pose.x * extrinsics.at<double>(1, 0) + pose.y * extrinsics.at<double>(1, 1)
	              + pose.z * extrinsics.at<double>(1, 2) + extrinsics.at<double>(1, 3);
	poseRobot.z = pose.x * extrinsics.at<double>(2, 0) + pose.y * extrinsics.at<double>(2, 1)
	              + pose.z * extrinsics.at<double>(2, 2) + extrinsics.at<double>(2, 3);

	return poseRobot;
}

VSPose convertSensor2RobotCoords(const VsCameraIntrinsics& intrinsics,
                                 const cv::Mat_<double>& extrinsics,
                                 const cv::Point& point,
                                 float zInCameraCoords)
{
	pcl::PointXYZ camCoordPoint = convertSensor2CamCoords(intrinsics, point, zInCameraCoords);
	VSPose poseCamCoord;
	poseCamCoord.x = camCoordPoint.x;
	poseCamCoord.y = camCoordPoint.y;
	poseCamCoord.z = camCoordPoint.z;
	return convertCam2RobotCoords(poseCamCoord, extrinsics);
}

cv::Mat convertRobot2CamCoords(const VSPose& pose, const cv::Mat_<double>& extrinsics)
{
	cv::Mat Rx = (cv::Mat_<double>(3, 3) << 1,
	              0,
	              0,
	              0,
	              std::cos(pose.roll),
	              -std::sin(pose.roll),
	              0,
	              std::sin(pose.roll),
	              cos(pose.roll));
	cv::Mat Ry = (cv::Mat_<double>(3, 3) << std::cos(pose.pitch),
	              0,
	              std::sin(pose.pitch),
	              0,
	              1,
	              0,
	              -std::sin(pose.pitch),
	              0,
	              std::cos(pose.pitch));
	cv::Mat Rz = (cv::Mat_<double>(3, 3) << std::cos(pose.yaw),
	              -std::sin(pose.yaw),
	              0,
	              std::sin(pose.yaw),
	              std::cos(pose.yaw),
	              0,
	              0,
	              0,
	              1);
	cv::Mat rotInRobotCoords = Rz * Ry * Rx;  // sequence z-y-x

	cv::Mat poseInRobotCoords = cv::Mat::eye(4, 4, CV_32F);
	rotInRobotCoords.copyTo(poseInRobotCoords(cv::Rect(0, 0, 3, 3)));
	poseInRobotCoords.at<float>(0, 3) = pose.x;
	poseInRobotCoords.at<float>(1, 3) = pose.y;
	poseInRobotCoords.at<float>(2, 3) = pose.z;

	// building the transformation from robot to camera coordinate system
	cv::Mat extrinsics4x4Robot2Cam = cv::Mat::eye(4, 4, CV_32F);
	cv::Mat extrRotRobot2Cam = extrinsics(cv::Rect(0, 0, 3, 3)).t();  // inverse rotation R_inv
	extrRotRobot2Cam.copyTo(extrinsics4x4Robot2Cam(cv::Rect(0, 0, 3, 3)));
	cv::Mat translationRobot2Cam = -extrRotRobot2Cam * extrinsics(cv::Rect(3, 0, 1, 3));  // translation = -R_inv*t
	translationRobot2Cam.copyTo(extrinsics4x4Robot2Cam(cv::Rect(3, 0, 1, 3)));

	cv::Mat poseInCamCoords = extrinsics4x4Robot2Cam * poseInRobotCoords;
	return poseInCamCoords;
}

cv::Mat convertPoseEuler2Mat(const VSPose& pose)
{
	cv::Mat Rx = (cv::Mat_<double>(3, 3) << 1,
	              0,
	              0,
	              0,
	              std::cos(pose.roll),
	              -std::sin(pose.roll),
	              0,
	              std::sin(pose.roll),
	              cos(pose.roll));
	cv::Mat Ry = (cv::Mat_<double>(3, 3) << std::cos(pose.pitch),
	              0,
	              std::sin(pose.pitch),
	              0,
	              1,
	              0,
	              -std::sin(pose.pitch),
	              0,
	              std::cos(pose.pitch));
	cv::Mat Rz = (cv::Mat_<double>(3, 3) << std::cos(pose.yaw),
	              -std::sin(pose.yaw),
	              0,
	              std::sin(pose.yaw),
	              std::cos(pose.yaw),
	              0,
	              0,
	              0,
	              1);
	cv::Mat rotMat = Rz * Ry * Rx;  // sequence z-y-x
	return rotMat;
}

float calcEuclidianDistanceCamCoords(const cv::Point& point0,
                                     const float depth0CamCooords,
                                     const cv::Point& point1,
                                     const float depth1CamCooords,
                                     const VsCameraIntrinsics& intrinsics)
{
	pcl::PointXYZ camCoordPoint0 = convertSensor2CamCoords(intrinsics, point0, depth0CamCooords);
	pcl::PointXYZ camCoordPoint1 = convertSensor2CamCoords(intrinsics, point1, depth1CamCooords);
	return pcl::euclideanDistance(camCoordPoint0, camCoordPoint1);
}

pcl::PointXYZ calcCenterOfPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	// TODO(aschaefer): check if this can be done faster with std::reduce
	double sumX = 0.0;
	double sumY = 0.0;
	double sumZ = 0.0;
	for (int i = 0; i < cloud->points.size(); ++i)
	{
		sumX += cloud->points[i].x;
		sumY += cloud->points[i].y;
		sumZ += cloud->points[i].z;
	}
	pcl::PointXYZ centerPointCloud;
	centerPointCloud.x = sumX / cloud->points.size();
	centerPointCloud.y = sumY / cloud->points.size();
	centerPointCloud.z = sumZ / cloud->points.size();
	return centerPointCloud;
}

cv::Point getCenterPoint(const std::vector<cv::Point2f>& points, const std::vector<int>& pointIdxs)
{
	cv::Point centerPoint;
	centerPoint.x = 0;
	centerPoint.y = 0;
	for (int i = 0; i < pointIdxs.size(); ++i)
	{
		assert(pointIdxs.at(i) >= 0 && pointIdxs.at(i) < points.size()
		       && "Index out of bounds in getCenterPoint. Check pointIdxs!");
		centerPoint.x += points.at(pointIdxs.at(i)).x;
		centerPoint.y += points.at(pointIdxs.at(i)).y;
	}
	centerPoint.x /= pointIdxs.size();
	centerPoint.y /= pointIdxs.size();

	return centerPoint;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr applySORFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	int mean_k = 50;
	double std_dev_mul_thresh = 0.5;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Create a statistical outlier filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(std_dev_mul_thresh);
    sor.filter(*cloud_filtered);

    return cloud_filtered;
}

}  // namespace math
