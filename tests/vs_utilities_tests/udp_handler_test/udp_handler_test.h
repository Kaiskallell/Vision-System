/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef UDP_HANDLER_TEST_H
#define UDP_HANDLER_TEST_H
#include "gtest/gtest.h"

class UdpHandlerTest : public ::testing::Test
{
  protected:
	UdpHandlerTest();

	~UdpHandlerTest() override;

	void SetUp() override;

	void TearDown() override;
};

#endif  // UDP_HANDLER_TEST_H
