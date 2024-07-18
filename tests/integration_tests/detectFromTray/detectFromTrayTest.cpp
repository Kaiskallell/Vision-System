#include "detectFromTrayTest.hpp"

#include "GetStatus.pb.h"
#include "vs_signalHandler.h"

namespace visionsystemsignals
{
extern bool shutdownProgam;  // defined in vs_signalHandler.h
}

DetectFromTrayTest::DetectFromTrayTest()
    : DetectFromTray("../../resources/testfiles/depthGanCamera/pickCamera.json",
                     "../../resources/testfiles/depthGanCamera/format.json",
                     "../../resources/testfiles/depthGanCamera/visionSystemConfig.json")
{
}

void DetectFromTrayTest::run()
{
	std::vector<VSPose> posesInRobotCoordinates;
	std::vector<int> classIds;

	getFrame(m_frame);  // gets the frame (color+ depth) out of the camera
	if (m_frame.colorImage.empty())
	{
		m_kLogger->warn("m_frame.colorImage.empty()");
		return;
	}
	eProductsSituationVSComm prodSituation = eProductsSituationVSComm::eProductsSituationVSComm_AVAILABLE;
	detectPosesWithNetworks(posesInRobotCoordinates, classIds, prodSituation);  // makes the inference of yolo and dlc
	if (posesInRobotCoordinates.size() == 0)  // no pose could be calculated with the given detections
	{
		// All detections lead to bad poses so we need to take a new frame
		m_kLogger->warn("posesInRobotCoordinates.size() == 0");
		return;
	}
	writePosesToDB(m_dbFacade,
	               posesInRobotCoordinates,
	               classIds,
	               prodSituation,
	               m_frameNumber,
	               m_rotEncoderValue);  // makes the poses accessible in vsCommunication
}

TEST_F(DetectFromTrayTest, calculateOnePose)
{
	constexpr float kToleranceInMeters = 0.004F;
	run();
	std::string serializedSceneInfos = m_dbFacade->getSerializedObjInfosFromDB();
	VMS::VisionSystem::GetStatusResponseAck getStatusResponseAck;
	getStatusResponseAck.ParseFromString(serializedSceneInfos);
	float zCoordinate = getStatusResponseAck.area_infos(0).product_infos(0).coord_z();
	constexpr float kLAndRUnitFactor = 10000;
	float expectedZCoordinate = m_constZValue * kLAndRUnitFactor;
	EXPECT_NEAR(zCoordinate, expectedZCoordinate, kToleranceInMeters);
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}