/**
 * @copyright Copyright (c) 2020 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * @brief Test for Yolo
 *
 */
#ifndef YOLO_TEST_H
#define YOLO_TEST_H

#include "gtest/gtest.h"
#include "yolo.h"

class YoloTest : public ::testing::Test
{
  protected:
	YoloTest();
	std::shared_ptr<yolo::Yolo> m_yolo;
};

#endif