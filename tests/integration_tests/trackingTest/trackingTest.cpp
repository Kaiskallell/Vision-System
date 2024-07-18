#include "trackingTest.h"

#include "GetStatus.pb.h"
#include "miscellaneous/miscellaneous.h"
#include "projectPaths.h"
#include "thread"
#include "vs_signalHandler.h"

TrackingTest::TrackingTest()
    : ObjTracking(utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/tracking/networksConfigAreaPlace.json",
                  utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/tracking/visionSystemConfig.json",
                  utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/tracking/pickCamera.json")
{
	fs::path visionSystemConfigPath =
	    utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/tracking/visionSystemConfig.json";
	m_dbFacade = std::make_shared<db::DbFacade>(visionSystemConfigPath);
	fs::path testFilePath =
	    utils::getHomeDir() / "Downloads/00_unitTestPipeline/testfiles/tracking/trakingTestData_twoBoxes.yml";
	m_kLogger->debug("loading testFileForTracking");
	m_fstorage = cv::FileStorage(testFilePath, cv::FileStorage::READ);
	m_kLogger->debug("finished loading testFileForTracking");
}

TrackingTest::~TrackingTest() { m_fstorage.release(); }

bool TrackingTest::getNewDataFromDetectionThread(const std::shared_ptr<InterfaceDetTrk> interfaceDetTrk,
                                                 cv::Mat& img,
                                                 std::vector<VSPose>& poses,
                                                 std::vector<int>& classIds,
                                                 uint64_t& frameNumber,
                                                 uint32_t& rotEncoderValue,
                                                 uint32_t& areaNumber,
                                                 TimeStamp& timeStamp,
                                                 bool& frameUpdated)
{
	if (m_dataCounter > 420)
	{
		m_dataCounter = 0;
		m_firstUpdate = true;
		++m_testIterations;
	}

	std::string key = "_" + std::to_string(m_dataCounter);
	std::cout << "key = " << key << std::endl;

	int frameUpdatedHelper = 0;
	frameUpdatedHelper = (int)m_fstorage[key]["frameUpdated"];
	if (frameUpdatedHelper == 1)
	{
		frameUpdated = true;
	}
	else
	{
		frameUpdated = false;
	}

	if (frameUpdated == true)
	{
		areaNumber = (int)m_fstorage[key]["areaNumber"];
		frameNumber = (int)m_fstorage[key]["frameNumber"];
		img = m_fstorage[key]["img"].mat();
		timeStamp.year = (int)m_fstorage[key]["Timestamp"]["year"];
		timeStamp.month = (int)m_fstorage[key]["Timestamp"]["month"];
		timeStamp.day = (int)m_fstorage[key]["Timestamp"]["day"];
		timeStamp.hour = (int)m_fstorage[key]["Timestamp"]["hour"];
		timeStamp.minute = (int)m_fstorage[key]["Timestamp"]["minute"];
		timeStamp.second = (int)m_fstorage[key]["Timestamp"]["second"];
		timeStamp.millisecond = (int)m_fstorage[key]["Timestamp"]["millisecond"];

		cv::FileNode classIdNode = m_fstorage[key]["classIds"];
		classIds.clear();
		cv::FileNodeIterator it = classIdNode.begin(), it_end = classIdNode.end();  // Go through the node
		for (; it != it_end; ++it)
		{
			classIds.emplace_back((int)*it);
		}

		cv::FileNode posesNode = m_fstorage[key]["poses"];
		poses.clear();
		it = posesNode.begin(), it_end = posesNode.end();  // Go through the node
		for (; it != it_end; ++it)
		{
			VSPose pose;
			pose.x = (float)(*it)["x"];
			pose.y = (float)(*it)["y"];
			pose.z = (float)(*it)["z"];
			pose.roll = (float)(*it)["roll"];
			pose.pitch = (float)(*it)["pitch"];
			pose.yaw = (float)(*it)["yaw"];
			poses.emplace_back(pose);
		}
	}
	else
	{
		m_lastTimestamp = timeStamp;
		areaNumber = (int)m_fstorage[key]["areaNumber"];
		timeStamp.year = (int)m_fstorage[key]["Timestamp"]["year"];
		timeStamp.month = (int)m_fstorage[key]["Timestamp"]["month"];
		timeStamp.day = (int)m_fstorage[key]["Timestamp"]["day"];
		timeStamp.hour = (int)m_fstorage[key]["Timestamp"]["hour"];
		timeStamp.minute = (int)m_fstorage[key]["Timestamp"]["minute"];
		timeStamp.second = (int)m_fstorage[key]["Timestamp"]["second"];
		timeStamp.millisecond = (int)m_fstorage[key]["Timestamp"]["millisecond"];
	}

	if (m_firstUpdate)
	{
		rotEncoderValue = (int)m_fstorage[key]["rotEncoderValue"];
		m_lastRotEncoderValue = rotEncoderValue;
		m_firstUpdate = false;
	}
	else
	{
		m_lastRotEncoderValue = rotEncoderValue;
		rotEncoderValue = (int)m_fstorage[key]["rotEncoderValue"];
	}

	if (m_dataCounter == 0)
	{
		m_lastTimestamp = timeStamp;
		if (timeStamp.millisecond > 100)
		{
			m_lastTimestamp.millisecond = timeStamp.millisecond - 100;
		}

		if (timeStamp.millisecond < 100)
		{
			throw std::runtime_error("timestamp does not fit for endless loop");
		}
	}

	m_dataCounter++;

	m_kLogger->debug("frameNumber = {}", frameNumber);
	m_kLogger->debug("rotEncoderValue = {}", rotEncoderValue);
	m_kLogger->debug("areaNumber = {}", areaNumber);
	m_kLogger->debug("frameUpdated = {}", frameUpdated);

	for (int i = 0; i < poses.size(); ++i)
	{
		m_kLogger->debug("poses.at(i).x = {}", poses.at(i).x);
		m_kLogger->debug("poses.at(i).y = {}", poses.at(i).y);
		m_kLogger->debug("poses.at(i).z = {}", poses.at(i).z);
		m_kLogger->debug("poses.at(i).roll  = {}", poses.at(i).roll);
		m_kLogger->debug("poses.at(i).pitch = {}", poses.at(i).pitch);
		m_kLogger->debug("poses.at(i).yaw  = {}", poses.at(i).yaw);
	}

	m_kLogger->debug("timeStamp.year = {}", timeStamp.year);
	m_kLogger->debug("timeStamp.month = {}", timeStamp.month);
	m_kLogger->debug("timeStamp.day = {}", timeStamp.day);
	m_kLogger->debug("timeStamp.hour = {}", timeStamp.hour);
	m_kLogger->debug("timeStamp.minute  = {}", timeStamp.minute);
	m_kLogger->debug("timeStamp.second  = {}", timeStamp.second);
	m_kLogger->debug("timeStamp.millisecond  = {}", timeStamp.millisecond);

	return true;
}

void TrackingTest::run(const std::shared_ptr<InterfaceDetTrk> interfaceDetTrk)
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
			m_kLogger->debug("frameUpdated && !poses.empty()");
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

TEST_F(TrackingTest, firstTrackingTest)
{
	m_dbFacade->flushDB();
	m_dbFacade->setShutDownProcesses(false);
	m_frameNumber = 0;
	std::shared_ptr<InterfaceDetTrk> interfaceDetTrk = std::make_shared<InterfaceDetTrk>();
	std::thread trackingThread(&TrackingTest::run, this, interfaceDetTrk);
	sleep(6);

	m_dbFacade->setShutDownProcesses(true);
	if (trackingThread.joinable())
	{
		trackingThread.join();
	}

	uint64_t areaNumber = 4;
	const size_t areaInfIdx = 0;

	// read results from database
	std::string serializedSceneInfos =
	    m_dbFacade->getSerializedObjInfosFromDB(areaNumber);  // these are the infos from detectPickable

	if (serializedSceneInfos.empty())
	{
		m_kLogger->error("serializedSceneInfos are empty. Maybe wrong areaNumber in test?");
	}

	VMS::VisionSystem::GetStatusResponseAck getStatusResponseAck;
	getStatusResponseAck.ParseFromString(serializedSceneInfos);
	areaNumber = getStatusResponseAck.area_infos(areaInfIdx).area_number();
	uint64_t frameUid = getStatusResponseAck.area_infos(areaInfIdx).frame_uid();
	for (int i = 0; i < getStatusResponseAck.area_infos(areaInfIdx).object_infos_size(); i++)
	{
		int32_t coord_x = getStatusResponseAck.area_infos(areaInfIdx).object_infos(i).coord_x();
		int32_t coord_y = getStatusResponseAck.area_infos(areaInfIdx).object_infos(i).coord_y();
		int32_t coord_z = getStatusResponseAck.area_infos(areaInfIdx).object_infos(i).coord_z();
		int32_t roll = getStatusResponseAck.area_infos(areaInfIdx).object_infos(i).angle_roll();
		int32_t pitch = getStatusResponseAck.area_infos(areaInfIdx).object_infos(i).angle_pitch();
		int32_t yaw = getStatusResponseAck.area_infos(areaInfIdx).object_infos(i).angle_yaw();

		uint32_t product_id = getStatusResponseAck.area_infos(areaInfIdx).object_infos(i).object_id();  // aka classId
		uint64_t product_uid = getStatusResponseAck.area_infos(areaInfIdx).object_infos(i).object_uid();

		m_kLogger->debug("areaNumber = {}, frame_uid = {},  product_uid = {}, product_id = {}, coord_x = {}, coord_y= "
		                 "{}, coord_z = {}, "
		                 "roll = {}, pitch "
		                 "= {}, yaw = {}",
		                 areaNumber,
		                 frameUid,
		                 product_uid,
		                 product_id,
		                 coord_x,
		                 coord_y,
		                 coord_z,
		                 roll,
		                 pitch,
		                 yaw);
		EXPECT_NE(product_uid, 1500000 + m_testIterations + i);
	}
	EXPECT_NE(getStatusResponseAck.area_infos(areaInfIdx).object_infos().size(), 0);
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
