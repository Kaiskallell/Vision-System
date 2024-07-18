/**
 * @copyright Copyright (c) 2020 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * @brief vs_communication getStatus test
 */

#include "../../vs_communication/vs_c_status.h"
#include "gtest/gtest.h"

TEST(VsCommunicationTest, get_status_test)
{
	std::string s = "";  // empty message required
	std::string r;
	GetStatus getStatus;

	int nReturn = getStatus.processGetStatus(s, r);

	EXPECT_EQ(nReturn, true);
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}