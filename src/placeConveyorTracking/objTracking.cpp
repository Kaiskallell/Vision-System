/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved

 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 *
 */

#include "objTracking.h"

#include <thread>

#include "miscellaneous/miscellaneous.h"
#include "spdlog/fmt/ostr.h"
namespace visionsystemsignals
{
extern bool shutdownProgam;  // defined in vs_signalHandler.h
}

ObjTracking::ObjTracking(const fs::path& networksConfigPath,
                         const fs::path& visionSystemConfigPath,
                         const fs::path& cameraConfigPath)

{
	m_dbFacade = std::make_shared<db::DbFacade>(visionSystemConfigPath);
	nlohmann::json jsonFile = utils::loadJsonFile(networksConfigPath);
	std::string placeConveyorWhichAxis = "xAxis";
	try
	{
		m_distanceThreshold = jsonFile.at("appConfigs").at("tracking").at("distanceThresholdHunAlgo").get<float>();
		m_nFramesExpirationThreshold = jsonFile.at("appConfigs").at("tracking").at("nFramesExpirationThreshold").get<size_t>();
		m_removeTrkIfCoordExceedsVal = jsonFile.at("appConfigs").at("tracking").at("removeTrkIfCoordExceedsVal").get<float>();
		m_conveyorVelocityDirection =
	    	jsonFile.at("appConfigs").at("tracking").at("conveyorVelocityDirection").get<float>();  // either -1.0 or 1.0
		m_conveyorVelocityFeedingFactor =
	    	jsonFile.at("appConfigs").at("tracking").at("conveyorVelocityFeedingFactor").get<float>();  // Vorschub
		m_conveyorEncoderIncrements =
	    	jsonFile.at("appConfigs").at("tracking").at("conveyorEncoderIncrements").get<float>();  // Inkrements of rot encoder
		placeConveyorWhichAxis =
	    	jsonFile.at("appConfigs").at("tracking").at("whichAxis").get<std::string>();  // (placeConveyor is along x or y axis)
	}
	catch (const std::exception& e)
	{
		m_dbFacade->writeErrorCodeToDB(VS_ERR_FORMAT_GENERALPARAMETERS_PLACE);
		m_kLogger->error("Could not read parameters from file: " + networksConfigPath.string());
		return;
	}
	
	m_placeCoordIdx = (placeConveyorWhichAxis == "xAxis") ? 0 : 3;   // x coord is 0, y coord is 3

	// for showing imgs with tracked objs
	constexpr int kMagicOnlyOneTrkCamIdx = 0;
	std::string serialNumber;
	float cropX = 0.0;
	float cropY = 0.0;
	try
	{
		cropX = jsonFile.at("cameraConfigs").at("camera").at(kMagicOnlyOneTrkCamIdx).at("cropX").get<float>();
		cropY = jsonFile.at("cameraConfigs").at("camera").at(kMagicOnlyOneTrkCamIdx).at("cropY").get<float>();
		serialNumber = jsonFile.at("cameraConfigs")
	                              	.at("camera")
	                               	.at(kMagicOnlyOneTrkCamIdx)
	                               	.at("serialNumbers")
	                               	.at(0)
	                               	.get<std::string>();
	}
	catch (const std::exception& e)
	{
		m_dbFacade->writeErrorCodeToDB(VS_ERR_CAMERA_INIT_ERROR_PLACE);
		m_kLogger->error("Could not read parameters from file: " + networksConfigPath.string());
		return;
	}
	
	int camConfIdx = utils::getCamIdxFromCameraConfig(serialNumber, cameraConfigPath);
	// read intrinsic paramters from camera.json
	nlohmann::json jsonFileCamConf = utils::loadJsonFile(cameraConfigPath);
	try
	{
		m_intrinsics.fx = jsonFileCamConf.at("camera")[camConfIdx].at("fx").get<float>();
		m_intrinsics.fy = jsonFileCamConf.at("camera")[camConfIdx].at("fy").get<float>();
		m_intrinsics.cx = jsonFileCamConf.at("camera")[camConfIdx].at("cx").get<float>();
		m_intrinsics.cy = jsonFileCamConf.at("camera")[camConfIdx].at("cy").get<float>();
		m_intrinsics.k1 = jsonFileCamConf.at("camera")[camConfIdx].at("distCoeffs")[0].get<double>();
		m_intrinsics.k2 = jsonFileCamConf.at("camera")[camConfIdx].at("distCoeffs")[1].get<double>();
		m_intrinsics.p1 = jsonFileCamConf.at("camera")[camConfIdx].at("distCoeffs")[2].get<double>();
		m_intrinsics.p2 = jsonFileCamConf.at("camera")[camConfIdx].at("distCoeffs")[3].get<double>();
		m_intrinsics.k3 = jsonFileCamConf.at("camera")[camConfIdx].at("distCoeffs")[4].get<double>();
	}
	catch (const std::exception& e)
	{
		m_dbFacade->writeErrorCodeToDB(VS_ERR_CAMERA_INIT_ERROR_PLACE);
		m_kLogger->error("Could not read parameters from file: " + cameraConfigPath.string());
		return;
	}
	

	m_intrinsics.cx = m_intrinsics.cx - cropX;
	m_intrinsics.cy = m_intrinsics.cy - cropY;

	m_extrinsics = cv::Mat_<double>(3, 4);
	try
	{
		m_extrinsics.at<double>(0, 0) = jsonFileCamConf.at("camera")[camConfIdx].at("r_11").get<double>();
		m_extrinsics.at<double>(0, 1) = jsonFileCamConf.at("camera")[camConfIdx].at("r_12").get<double>();
		m_extrinsics.at<double>(0, 2) = jsonFileCamConf.at("camera")[camConfIdx].at("r_13").get<double>();
		m_extrinsics.at<double>(1, 0) = jsonFileCamConf.at("camera")[camConfIdx].at("r_21").get<double>();
		m_extrinsics.at<double>(1, 1) = jsonFileCamConf.at("camera")[camConfIdx].at("r_22").get<double>();
		m_extrinsics.at<double>(1, 2) = jsonFileCamConf.at("camera")[camConfIdx].at("r_23").get<double>();
		m_extrinsics.at<double>(2, 0) = jsonFileCamConf.at("camera")[camConfIdx].at("r_31").get<double>();
		m_extrinsics.at<double>(2, 1) = jsonFileCamConf.at("camera")[camConfIdx].at("r_32").get<double>();
		m_extrinsics.at<double>(2, 2) = jsonFileCamConf.at("camera")[camConfIdx].at("r_33").get<double>();

		// offsets
		m_extrinsics.at<double>(0, 3) = jsonFileCamConf.at("camera")[camConfIdx].at("x_offset").get<double>();
		m_extrinsics.at<double>(1, 3) = jsonFileCamConf.at("camera")[camConfIdx].at("y_offset").get<double>();
		m_extrinsics.at<double>(2, 3) = jsonFileCamConf.at("camera")[camConfIdx].at("z_offset").get<double>();
	}
	catch (const std::exception& e)
	{
		m_dbFacade->writeErrorCodeToDB(VS_ERR_CAMERA_INIT_ERROR_PLACE);
		m_kLogger->error("Could not read parameters from file: " + cameraConfigPath.string());
		return;
	}
	m_moduleConfigured = true;
}

ObjTracking::~ObjTracking() {}

bool ObjTracking::getNewDataFromDetectionThread(const std::shared_ptr<InterfaceDetTrk> interfaceDetTrk,
                                                cv::Mat& img,
                                                std::vector<VSPose>& poses,
                                                std::vector<int>& classIds,
                                                uint64_t& frameNumber,
                                                uint32_t& rotEncoderValue,
                                                uint32_t& areaNumber,
                                                TimeStamp& timeStamp,
                                                bool& frameUpdated)
{
	interfaceDetTrk->lockInterface();
	if (m_currentTimestamp.millisecond == interfaceDetTrk->getNewDatafromDetTimestamp().millisecond)
	{
		// no new timestamp available
		interfaceDetTrk->unlockInterface();
		return false;
	}

	frameUpdated = interfaceDetTrk->getFrameUpdated();
	if (frameUpdated == true)
	{
		m_lastTimestamp = timeStamp;
		timeStamp = interfaceDetTrk->getNewDatafromDetTimestamp();
		img = interfaceDetTrk->getImg();
		poses = interfaceDetTrk->getNewData();
		classIds = interfaceDetTrk->getClassIds();
		frameNumber = interfaceDetTrk->getFrameNumber();
		areaNumber = interfaceDetTrk->getAreaNumber();
	}
	else
	{
		m_lastTimestamp = timeStamp;
		timeStamp = interfaceDetTrk->getNewDatafromDetTimestamp();
		areaNumber = interfaceDetTrk->getAreaNumber();
	}

	// the initial value leads to non sensical velocity data
	if (m_firstUpdate)
	{
		rotEncoderValue = interfaceDetTrk->getRotEncoderVal();
		m_lastRotEncoderValue = rotEncoderValue;  // velocity gets calculated to 0 for the first value
		m_firstUpdate = false;
	}
	else
	{
		// normal case
		m_lastRotEncoderValue = rotEncoderValue;
		rotEncoderValue = interfaceDetTrk->getRotEncoderVal();
	}

	interfaceDetTrk->unlockInterface();

	return true;
}

void ObjTracking::updateStateWithEncoder(std::vector<TrackedObject>& trks, uint64_t frameNumber)
{
	for (int i = 0; i < trks.size(); ++i)  // max number of tracking is 15
	{
		if (!trks.at(i).m_state.m_active)
		{
			continue;
		}

		int64_t encoderDifference = (int64_t)m_currentRotEncoderValue - (int64_t)m_lastRotEncoderValue;
		constexpr uint32_t kUnderflowThreshold = 4194304;  // 2^22 = 4194304
		constexpr uint32_t kOverflowThreshold = 67108864;  // 2^26 = 67108864
		constexpr uint32_t kMaxEncoderValue = 268435456;   // 2^28 = 268435456
		if (m_currentRotEncoderValue < kUnderflowThreshold && m_lastRotEncoderValue > kOverflowThreshold)

		{
			encoderDifference = m_currentRotEncoderValue + kMaxEncoderValue - m_lastRotEncoderValue;
		}
		else if (m_lastRotEncoderValue < kUnderflowThreshold && m_currentRotEncoderValue > kOverflowThreshold)
		{
			encoderDifference = m_currentRotEncoderValue - kMaxEncoderValue - m_lastRotEncoderValue;
		}

		const float displacementAccordingToEncoder = encoderDifference * m_conveyorVelocityDirection
		                                             * m_conveyorVelocityFeedingFactor / m_conveyorEncoderIncrements;
		trks.at(i).m_state.m_stateX[0] = trks.at(i).m_state.m_stateXOld[0];
		trks.at(i).m_state.m_stateX[3] = trks.at(i).m_state.m_stateXOld[3];

		if (m_placeCoordIdx == 0)
		{
			trks.at(i).m_state.m_stateX[0] += displacementAccordingToEncoder;
		}
		else
		{
			trks.at(i).m_state.m_stateX[3] += displacementAccordingToEncoder;
		}

		trks.at(i).m_state.m_frameNumber = frameNumber;
	}
}

float ObjTracking::calculateDistance(const VSPose& newPose, const TrackedObject& trk)
{
	// m_kLogger->debug("trk.m_state.m_state.m_id = {}", trk.m_state.m_id);
	// m_kLogger->debug("trk.m_state.m_stateX.hasNaN() = {}", trk.m_state.m_stateX.hasNaN());
	// m_kLogger->debug("trk.m_state.m_stateX = {}", trk.m_state.m_stateX);

	// m_kLogger->debug("newPose.x = {}", newPose.x);
	// m_kLogger->debug("trk.m_state.m_stateX(0, 0) = {}", trk.m_state.m_stateX(0, 0));
	// m_kLogger->debug("newPose.y = {}", newPose.y);
	// m_kLogger->debug("trk.m_state.m_stateX(3, 0) = {}", trk.m_state.m_stateX(3, 0));

	if (trk.m_state.m_active == false || trk.m_state.m_stateX.hasNaN())
	{
		return 0.0f;
	}

	// calc euclidian distance between new pose and tracked obj
	float distance = std::sqrt((newPose.x - trk.m_state.m_stateX(0, 0)) * (newPose.x - trk.m_state.m_stateX(0, 0))
	                           + (newPose.y - trk.m_state.m_stateX(3, 0)) * (newPose.y - trk.m_state.m_stateX(3, 0)));
	if (distance < 0.000001)
	{
		distance = 0.000002;
	}

	return distance;
}

// TODO(aschaefer): replace hard coded sizes of detections and tracks (10 and 15)
Eigen::Matrix<int, 10, 1> ObjTracking::hungarianMatching(const std::vector<VSPose>& newPoses,
                                                         const Eigen::Matrix<long double, 10, 15>& distanceMatrix)
{
	Eigen::Matrix<int, 10, 1> matchedIndices;
	matchedIndices.setZero();  // reset Indices

	// set value to one for each value which is bigger than the threshold
	Eigen::Matrix<int, 10, 15> distanceMatrixBool;
	// distanceMatrixBool = (distanceMatrix.array() > 0.0 && distanceMatrix.array() <
	// m_distanceThreshold).cast<float>();
	for (int r = 0; r < distanceMatrixBool.rows(); ++r)
	{
		for (int c = 0; c < distanceMatrixBool.cols(); ++c)
		{
			if (distanceMatrix(r, c) > 0.0000000000001 && distanceMatrix(r, c) < m_distanceThreshold)
			{
				distanceMatrixBool(r, c) = 1;
			}
			else
			{
				distanceMatrixBool(r, c) = 0;
			}
		}
	}

	if (distanceMatrixBool.rowwise().sum().maxCoeff() == 0)
	{
		// m_kLogger->debug("No Assignment necessary -> No Dets fits to m_trks ");
		return matchedIndices;
	}

	// check if each row and each col contains only one match
	if (distanceMatrixBool.rowwise().sum().maxCoeff() == 1.0 && distanceMatrixBool.colwise().sum().maxCoeff() == 1.0)
	{
		// create an vector with all maches (Vectorposition -> Detection, Vectorvalue -> Track)
		for (int i = 0; i < newPoses.size(); i++)
		{
			if (distanceMatrixBool.row(i).maxCoeff(&m_maxIndex) == 1.0)  // only if one Index fits (Value is 1)
			{
				matchedIndices(i, 0) = m_maxIndex + 1;  // add +1 because zero represents "no match"
			}
		}
	}
	else
	{
		// m_kLogger->debug("Assignment via IOU is not clear -> use linear assignment");
		// use extern implementation therfor -> convert Eigen Matrix in a 2D Vector
		std::vector<std::vector<double>> costMatrix(10, std::vector<double>(15, 0));
		// result should only present IOUs > threshold
		for (int i = 0; i < 10; i++)
		{
			for (int j = 0; j < 15; j++)
			{
				if (distanceMatrix(i, j) == 0.0f)
				{
					costMatrix[i][j] = 100000.0f;
				}
				else
				{
					costMatrix[i][j] = distanceMatrix(i, j);
				}
				// m_kLogger->debug("costMatrix:  i={},j={}, costMatrix={}", i, j, costMatrix[i][j]);
			}
		}

		std::vector<int> assignment(10, 0);
		double cost = m_hungAlgo.Solve(costMatrix, assignment);

		for (int i = 0; i < newPoses.size(); i++)
		{
			matchedIndices(i, 0) = assignment[i] + 1;  // add +1 because zero represents "no match"
		}

		// for (int i = 0; i < assignment.size(); i++)
		// {
		// 	m_kLogger->debug("assignment: {}: --> {}", i, assignment[i]);
		// }
	}
	return matchedIndices;
}

void ObjTracking::associatePosesToTrks(const std::vector<TrackedObject>& trks,
                                       const std::vector<VSPose>& poses,
                                       Eigen::Matrix<int, 10, 1>& matchedIndices,
                                       Eigen::Matrix<int, 10, 1>& unmatchedDetections)
{
	if (poses.size() == 0)
	{
		m_kLogger->error("no new poses for associatePosesToTrks");
	}

	Eigen::Matrix<long double, 10, 15> distanceMatrix;
	distanceMatrix.setZero();
	for (int i = 0; i < poses.size(); ++i)  // 10 detections are max
	{
		for (int j = 0; j < trks.size(); ++j)  // 15 tracks max
		{
			distanceMatrix(i, j) = calculateDistance(poses.at(i), trks.at(j));
		}
	}

	matchedIndices = hungarianMatching(poses, distanceMatrix);

	for (int i = 0; i < matchedIndices.size(); ++i)
	{
		const float matchedIdx = matchedIndices(i, 0);
		// m_kLogger->debug(
		//     "detection: {} --> track: {} track id: {}", i, matchedIdx, trks.at(matchedIndices(i, 0)).m_state.m_id);
	}
	// create list with unmatched dets and unmatched trks

	unmatchedDetections.setZero();
	int j = 0;
	for (int i = 0; i < poses.size(); i++)
	{
		if (matchedIndices(i, 0) == 0)
		{
			// found unmatched detection
			unmatchedDetections(j, 0) = i + 1;  //+1 because 0 is empty
			j += 1;
		}
	}

	m_unmatchedTrackers.setZero();
	j = 0;
	for (int i = 0; i < trks.size(); i++)
	{
		if (trks.at(i).m_state.m_active)
		{
			if (!(matchedIndices.array() == i + 1).any())
			{
				// found unmatched detection
				m_unmatchedTrackers(j, 0) = i + 1;  //+1 because 0 is empty
				j += 1;
			}
		}
	}
}

void ObjTracking::updateMatchedDets(const std::vector<VSPose>& newPoses,
                                    const std::vector<int>& classIds,
                                    const uint64_t frameNumber,
                                    const uint32_t rotEncoderValue,
                                    const uint32_t areaNumber,
                                    const Eigen::Matrix<int, 10, 1>& matchedIdxs,
                                    std::vector<TrackedObject>& trks)
{
	for (int i = 0; i < newPoses.size(); i++)
	{
		if (matchedIdxs(i, 0) > 0)  // zero is no match
		{
			int matchedIndice = matchedIdxs(i, 0) - 1;  //-1 -> reason see Function HungarianMatching
			trks[matchedIndice].m_state.m_stateX(0, 0) = newPoses.at(i).x;
			trks[matchedIndice].m_state.m_stateX(3, 0) = newPoses.at(i).y;
			trks[matchedIndice].m_state.m_zCoord = newPoses.at(i).z;
			trks[matchedIndice].m_state.m_yaw = newPoses.at(i).yaw;
			trks[matchedIndice].m_state.m_clss = classIds.at(i);
			trks[matchedIndice].m_state.m_areaNumber = areaNumber;
			trks[matchedIndice].m_state.m_frameNumber = frameNumber;
			trks[matchedIndice].m_state.m_rotEncoderValue = rotEncoderValue;
			trks[matchedIndice].m_frameSinceUpdate = 0;
		}
	}
}

void ObjTracking::removeDeadTrks(std::vector<TrackedObject>& trks, bool frameUpdated)
{
	// TODO(aschaefer): remove hard coded number of tracked objs
	for (int i = 0; i < 15; ++i)
	{
		if (frameUpdated && m_unmatchedTrackers(i, 0) > 0)
		{
			int unmatchedTrkIndice = m_unmatchedTrackers(i, 0) - 1;
			trks[unmatchedTrkIndice].m_frameSinceUpdate += 1;
		}

		bool shouldRemove = m_conveyorVelocityDirection < 0
		                        ? trks[i].m_state.m_stateX[m_placeCoordIdx]
		                              < m_removeTrkIfCoordExceedsVal  // negative x or y coord (0 or 3)
		                        : trks[i].m_state.m_stateX[m_placeCoordIdx]
		                              > m_removeTrkIfCoordExceedsVal;  // positiv x or y coord (0 or 3)

		if (trks[i].m_frameSinceUpdate > m_nFramesExpirationThreshold || shouldRemove)
		{
			trks[i].m_state.m_active = false;
		}
	}
}

void ObjTracking::initNewTrks(const std::vector<VSPose>& newPoses,
                              const std::vector<int>& classIds,
                              const Eigen::Matrix<int, 10, 1>& unmatchedDetections,
                              std::vector<TrackedObject>& trks,
                              const uint64_t frameNumber,
                              const uint32_t rotEncoderValue,
                              const uint32_t areaNumber)
{
	for (int i = 0; i < newPoses.size(); i++)
	{
		if (unmatchedDetections(i, 0) > 0)
		{
			int unmatchedDetIdx = unmatchedDetections(i, 0) - 1;
			// find next free store
			for (int j = 0; j < trks.size(); j++)
			{
				if (trks[j].m_state.m_active == false)
				{
					// reset all variable parameters
					trks[j].m_state.m_stateX.setZero();
					trks[j].m_frameSinceUpdate = 0;

					// int Trk parameter
					trks[j].m_state.m_stateX(0, 0) = newPoses.at(unmatchedDetIdx).x;
					trks[j].m_state.m_stateX(3, 0) = newPoses.at(unmatchedDetIdx).y;
					trks[j].m_state.m_zCoord = newPoses.at(unmatchedDetIdx).z;
					trks[j].m_state.m_yaw = newPoses.at(unmatchedDetIdx).yaw;
					trks[j].m_state.m_stateX(4, 0) = 0.0;
					trks[j].m_state.m_stateX(1, 0) = 0.0;
					trks[j].m_state.m_clss = classIds.at(unmatchedDetIdx);
					trks[j].m_state.m_id = m_counterTrackID;
					trks[j].m_state.m_active = true;
					trks[j].m_state.m_frameNumber = frameNumber;
					trks[j].m_state.m_rotEncoderValue = rotEncoderValue;
					trks[j].m_state.m_areaNumber = areaNumber;
					m_counterTrackID += 1;
					break;
				}
				if (j == 14)
				{
					m_kLogger->warn(" store for Tracks is now full");
				}
			}
		}
		else
		{
			// all unmatched Detections are found
			// m_kLogger->info("all unmatched Detections are found");
			break;
		}
	}
}

void ObjTracking::writeTrksToDB(const std::vector<TrackedObject>& trks)
{
	// m_kLogger->debug("trks.empty() = {}", trks.empty());
	if (trks.empty())
	{
		return;
	}
	std::vector<VSPose> poses;
	std::vector<int> classes;
	std::vector<uint64_t> trackingIds;
	int32_t rotEncoderValue = 0;
	uint64_t frameId = 0;
	uint32_t areaNumber = 0;
	for (const auto& trk : trks)
	{
		// m_kLogger->debug("trk.m_state.m_active = {}", trk.m_state.m_active);
		// m_kLogger->debug("trk.m_state.m_id = {}", trk.m_state.m_id);
		// m_kLogger->debug("trk.m_frameSinceUpdate = {}", trk.m_frameSinceUpdate);

		if (!trk.m_state.m_active)
		{
			continue;
		}
		VSPose pose;
		pose.x = trk.m_state.m_stateX(0, 0);
		pose.y = trk.m_state.m_stateX(3, 0);
		pose.z = trk.m_state.m_zCoord;
		pose.roll = 0.0;
		pose.pitch = 0.0;
		pose.yaw = trk.m_state.m_yaw;
		poses.emplace_back(pose);
		classes.emplace_back(trk.m_state.m_clss);
		trackingIds.emplace_back(trk.m_state.m_id);
		rotEncoderValue = trk.m_state.m_rotEncoderValue;
		frameId = trk.m_state.m_frameNumber;
		areaNumber = trk.m_state.m_areaNumber;
	}
	// m_kLogger->debug("poses.size() = {}", poses.size());
	// m_kLogger->debug("trks.at(0).m_state.m_areaNumber = {}", trks.at(0).m_state.m_areaNumber);
	// m_kLogger->debug("writing to db from tracking");

	m_dbFacade->serializeTrayInfosToDB(poses, classes, rotEncoderValue, frameId, trackingIds, areaNumber);
}

void ObjTracking::showTrks(const std::vector<TrackedObject>& trks, cv::Mat& debugColorImgTracking)
{
	for (int i = 0; i < trks.size(); ++i)
	{
		if (!trks.at(i).m_state.m_active)
		{
			continue;  // do not print old tracks
		}
		// show img for debugging purposes
		cv::Mat positionRobot = (cv::Mat_<double>(4, 1) << trks.at(i).m_state.m_stateX(0, 0),
		                         trks.at(i).m_state.m_stateX(3, 0),
		                         trks.at(i).m_state.m_zCoord,
		                         1.0);

		cv::Mat rotMat = m_extrinsics(cv::Rect(0, 0, 3, 3));
		cv::Mat offsetMat = m_extrinsics(cv::Rect(3, 0, 1, 3));

		cv::Mat tVecInv = -rotMat.t() * offsetMat;
		cv::Mat invExtrinsics = cv::Mat::eye(4, 4, CV_64FC1);

		cv::Mat invRotMat = rotMat.t();
		invRotMat.copyTo(invExtrinsics(cv::Rect(0, 0, 3, 3)));
		tVecInv.copyTo(invExtrinsics(cv::Rect(3, 0, 1, 3)));
		cv::Mat positionCam = invExtrinsics * positionRobot;

		int u = m_intrinsics.fx * positionCam.at<double>(0, 0) / positionCam.at<double>(2, 0) + m_intrinsics.cx;
		int v = m_intrinsics.fy * positionCam.at<double>(1, 0) / positionCam.at<double>(2, 0) + m_intrinsics.cy;
		// draw and show img with tracked poses
		cv::circle(debugColorImgTracking, cv::Point(u, v), 10, cv::Scalar(255, 0, 0), 5);
		std::string text = std::to_string(trks.at(i).m_state.m_id);
		cv::putText(debugColorImgTracking, text, cv::Point(u, v), 0, 1.2, cv::Scalar(0, 0, 255), 2);
	}

	misc::showImage("debugColorImgTracking", debugColorImgTracking);
}

void ObjTracking::run(const std::shared_ptr<InterfaceDetTrk> interfaceDetTrk)
{
	std::vector<TrackedObject> trks = std::vector<TrackedObject>(15, TrackedObject());

	std::vector<db::ImprintMap> imprintMappingTable;

	while (!visionsystemsignals::shutdownProgam && !m_dbFacade->getShutDownProcesses())
	{
		misc::waitKey1ms();
		std::vector<VSPose> poses;
		std::vector<int> classIds;
		uint64_t frameNumber = 0;
		uint32_t areaNumber = 0;
		cv::Mat debugColorImgTracking;
		bool frameUpdated = false;
		// get poses and timeStamp from other thread
		// m_kLogger->debug(" get poses and timeStamp from other thread");
		if (!getNewDataFromDetectionThread(interfaceDetTrk,
		                                   debugColorImgTracking,
		                                   poses,
		                                   classIds,
		                                   frameNumber,
		                                   m_currentRotEncoderValue,
		                                   areaNumber,
		                                   m_currentTimestamp,
		                                   frameUpdated))
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(2));  // timestamp is not updated
			continue;
		}

		if (frameUpdated && !poses.empty())
		{
			updateStateWithEncoder(trks, frameNumber);

			// matching with hungarian algorithm
			Eigen::Matrix<int, 10, 1> matchedIdxs;
			Eigen::Matrix<int, 10, 1> unmatchedDetections;
			associatePosesToTrks(trks, poses, matchedIdxs, unmatchedDetections);

			updateMatchedDets(poses, classIds, frameNumber, m_currentRotEncoderValue, areaNumber, matchedIdxs, trks);
			initNewTrks(poses, classIds, unmatchedDetections, trks, frameNumber, m_currentRotEncoderValue, areaNumber);
			m_firstDetection = false;
		}
		else
		{
			updateStateWithEncoder(trks, frameNumber);
		}

		for (int i = 0; i < trks.size(); ++i)  // max number of tracking is 15
		{
			if (!trks.at(i).m_state.m_active)
			{
				continue;
			}

			trks.at(i).m_state.m_stateXOld[0] = trks.at(i).m_state.m_stateX[0];
			trks.at(i).m_state.m_stateXOld[1] = trks.at(i).m_state.m_stateX[1];
			trks.at(i).m_state.m_stateXOld[2] = trks.at(i).m_state.m_stateX[2];
			trks.at(i).m_state.m_stateXOld[3] = trks.at(i).m_state.m_stateX[3];
			trks.at(i).m_state.m_stateXOld[4] = trks.at(i).m_state.m_stateX[4];
			trks.at(i).m_state.m_stateXOld[5] = trks.at(i).m_state.m_stateX[5];
			trks.at(i).m_state.m_rotEncoderValue = m_currentRotEncoderValue;
		}

		if (!m_firstDetection)
		{
			removeDeadTrks(trks, frameUpdated);
		}

		// makes the poses accessible in vsCommunication
		if (frameUpdated)
		{
			writeTrksToDB(trks);
			showTrks(trks, debugColorImgTracking);
		}
	}

}
