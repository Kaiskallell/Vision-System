/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "udp_handler_test.h"

#include "../include/vsUdp/udphandler.h"

UdpHandlerTest::UdpHandlerTest() {}

UdpHandlerTest::~UdpHandlerTest() {}

void UdpHandlerTest::SetUp() {}

void UdpHandlerTest::TearDown() {}

TEST_F(UdpHandlerTest, sendAndReceive) {}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
