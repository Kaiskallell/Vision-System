/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved

 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 *
 */

#ifndef OBJTRACKING_H
#define OBJTRACKING_H

#include <Hungarian.h>  // https://github.com/mcximing/hungarian-algorithm-cpp

#include <Eigen/Dense>

#include "cameraInterface.hpp"
#include "dbFacade/dbFacade.h"
#include "logging.h"
#include "projectPaths.h"
#include "trackingInterface.h"
#include "vs_image.hpp"
#include "vs_poseObject.h"
#include "vs_c_communicationError.h"

struct BoundingBox
{
	float m_x = 0.0f;
	float m_y = 0.0f;
	float m_w = 0.0f;
	float m_h = 0.0f;
};

struct DetectedObject
{
	BoundingBox m_bbox_px;
	BoundingBox m_bbox_world;
	float m_score = 0.0f;
	int m_clss = 0;
	bool m_active = false;
	int m_numberOfDets = 0;  // should only be used by first Object
};

struct StateTrackedObject
{
	StateTrackedObject() : m_stateX(Eigen::Matrix<float, 6, 1>::Zero()) {}

	Eigen::Matrix<float, 6, 1> m_stateX;  // x, vx, ax, y, vy, ay
	Eigen::Matrix<float, 6, 1> m_stateXOld = Eigen::Matrix<float, 6, 1>::Zero();
	float m_zCoord = 0.0f;
	float m_yaw = 0.0f;
	int m_clss = 0;
	int m_id = 0;
	bool m_active = false;

	uint64_t m_frameNumber = 0;
	uint32_t m_rotEncoderValue = 0;
	uint32_t m_areaNumber = 0;
};

struct TrackedObject  // TODO(aschaefer): make smart pointer and set to nullptr when inactive
{
	TrackedObject() : m_P(Eigen::Matrix<float, 6, 6>::Zero()) {}

	StateTrackedObject m_state;

	Eigen::Matrix<float, 6, 6> m_P;

	int m_hitStreak = 0;
	int m_frameSinceUpdate = 0;
};

class ObjTracking
{
  public:
	explicit ObjTracking(const fs::path& networksConfigPath,
	                     const fs::path& visionSystemConfigPath,
	                     const fs::path& cameraConfigPath);
	~ObjTracking();

	bool m_moduleConfigured = false;
	virtual void run(const std::shared_ptr<InterfaceDetTrk> interfaceDetTrk);

  protected:
	virtual bool getNewDataFromDetectionThread(const std::shared_ptr<InterfaceDetTrk> interfaceDetTrk,
	                                           cv::Mat& img,
	                                           std::vector<VSPose>& poses,
	                                           std::vector<int>& classIds,
	                                           uint64_t& frameNumber,
	                                           uint32_t& rotEncoderValue,
	                                           uint32_t& areaNumber,
	                                           TimeStamp& timeStamp,
	                                           bool& frameUpdated);

	void updateStateWithEncoder(std::vector<TrackedObject>& trks, uint64_t frameNumber);
	float calculateDistance(const VSPose& newPose, const TrackedObject& trk);
	Eigen::Matrix<int, 10, 1> hungarianMatching(const std::vector<VSPose>& newPoses,
	                                            const Eigen::Matrix<long double, 10, 15>& distanceMatrix);
	void associatePosesToTrks(const std::vector<TrackedObject>& trks,
	                          const std::vector<VSPose>& poses,
	                          Eigen::Matrix<int, 10, 1>& matchedIndices,
	                          Eigen::Matrix<int, 10, 1>& unmatchedDetections);
	void updateMatchedDets(const std::vector<VSPose>& newPoses,
	                       const std::vector<int>& classIds,
	                       const uint64_t frameNumber,
	                       const uint32_t rotEncoderValue,
	                       const uint32_t areaNumber,
	                       const Eigen::Matrix<int, 10, 1>& matchedIdxs,
	                       std::vector<TrackedObject>& trks);
	void removeDeadTrks(std::vector<TrackedObject>& trks, bool frameUpdated);
	void initNewTrks(const std::vector<VSPose>& newPoses,
	                 const std::vector<int>& classIds,
	                 const Eigen::Matrix<int, 10, 1>& unmatchedDetections,
	                 std::vector<TrackedObject>& trks,
	                 const uint64_t frameNumber,
	                 const uint32_t rotEncoderValue,
	                 const uint32_t areaNumber);
	void writeTrksToDB(const std::vector<TrackedObject>& trks);
	void showTrks(const std::vector<TrackedObject>& trks, cv::Mat& debugColorImgTracking);

	Eigen::MatrixXf::Index m_maxIndex;
	float m_distanceThreshold = 0.0f;
	Eigen::Matrix<int, 15, 1> m_unmatchedTrackers;
	// variables for matching with hungarian algorithm
	HungarianAlgorithm m_hungAlgo;
	size_t m_nFramesExpirationThreshold = 0;
	int m_counterTrackID = 0;
	bool m_firstDetection = true;
	bool m_firstUpdate = true;
	std::shared_ptr<db::DbFacade> m_dbFacade;  // handles data base interactions
	TimeStamp m_currentTimestamp;              // time where picture was triggerd
	TimeStamp m_lastTimestamp;                 // time where picture was take in the last iteration
	uint32_t m_currentRotEncoderValue;
	uint32_t m_lastRotEncoderValue;
	VsCameraIntrinsics m_intrinsics;
	cv::Mat m_extrinsics;
	float m_removeTrkIfCoordExceedsVal = 0.0f;
	float m_conveyorVelocityDirection = 0.0f;      // either -1.0 or 1.0
	float m_conveyorVelocityFeedingFactor = 0.0f;  // Vorschub
	float m_conveyorEncoderIncrements = 0.0f;      // Inkrements of rot encoder
	bool m_firstTime = true;  // for calculate the timediff at the first timestep
	int m_placeCoordIdx = 0;

	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("ObjTracking");
};

#endif
