/**
 * @copyright Copyright (c) 2020 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * @brief Test for cpp-deeplabcut
 *
 */
#ifndef DLC_TEST_H
#define DLC_TEST_H

#include "gtest/gtest.h"
#include "inferDLC.h"

class InferDLCTest : public ::testing::Test
{
  protected:
	InferDLCTest();
	std::shared_ptr<dlc::InferDLC> m_dlc;
};

#endif