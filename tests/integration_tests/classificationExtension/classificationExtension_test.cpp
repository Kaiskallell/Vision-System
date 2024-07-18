/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "classificationExtension_test.h"

#include <memory>

#include "gtest/gtest.h"
#include "projectPaths.h"
#include "tc3Mock.h"

ClassificationTest::ClassificationTest() {}
ClassificationTest::~ClassificationTest()
{
	m_dbFacade->setShutDownProcesses(true);
	m_classificatorExtensionProcess.terminate();
}

TEST_F(ClassificationTest, initialization)
{
	fs::path visionSystemConfigPath =
	    "/home/cobot/Downloads/00_unitTestPipeline/testfiles/classificatorExtension/visionSystemConfig.json";
	m_dbFacade = std::make_unique<db::DbFacade>(visionSystemConfigPath);
	m_dbFacade->setShutDownProcesses(false);

	// start classificationExtension
	boost::process::group m_classificatorExtensionGroup;
	fs::path networksConfigPath = "/home/cobot/Downloads/00_unitTestPipeline/testfiles/classificatorExtension/1/"
	                              "networksConfigClassificatorExtension.json";
	std::string networkConfigArg = "-networksConfigPath=" + networksConfigPath.string();
	fs::path cameraConfigPath =
	    "/home/cobot/Downloads/00_unitTestPipeline/testfiles/classificatorExtension/pickCamera.json";
	std::string camConfigArg = "--cameraConfigPath=" + cameraConfigPath.string();
	std::vector<std::string> classificatorArgs = {camConfigArg, networkConfigArg};

	fs::path classificatorExtensionPath = utils::getProjectRootDir() / "build/bin/classificatorExtension";

	m_classificatorExtensionProcess =
	    bp::child(classificatorExtensionPath.string(), bp::args(classificatorArgs), m_classificatorExtensionGroup);

	// start fake tc3 and save classId to file
	const std::string listeningIpAdress = "127.0.0.1";
	const size_t listeningUdpPort = 53005;
	const std::string sendingIpAdress = "127.0.0.1";
	const size_t sendingUdpPort = 53004;
	tc3mock::Tc3Mock mockTc3(listeningIpAdress, listeningUdpPort, sendingIpAdress, sendingUdpPort);
	int classId = -1;
	int frameCounter = 0;
	while (classId < 1)
	{
		if (mockTc3.receivedData())
		{
			VsToTc3Datagram dataFromVS = mockTc3.getReceiverData();
			classId = dataFromVS.sProductsSituation.classId;
			std::cout << "classId tc3 = " << classId << std::endl;
			frameCounter++;
			Tc3ToVsDatagram sendObj;
			sendObj.sCameraTrigger.uImageNumber = frameCounter;
			mockTc3.send(sendObj);
		}

		sleep(1);
	}

	// close all processes
	m_dbFacade->setShutDownProcesses(true);

	EXPECT_EQ(classId, 1);
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
