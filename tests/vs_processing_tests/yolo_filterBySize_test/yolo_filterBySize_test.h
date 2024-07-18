/**
 * @copyright Copyright (c) 2020 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * @brief Test for Yolo FilterBySize
 *
 */
#ifndef YOLO_FILTERBYSIZE_TEST_H
#define YOLO_FILTERBYSIZE_TEST_H

#include "gtest/gtest.h"
#include "yolo.h"

class YoloFilterBySizeTest : public ::testing::Test
{
  protected:
	YoloFilterBySizeTest();
	std::shared_ptr<yolo::Yolo> m_yolo;
};

#endif