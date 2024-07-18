/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef HTTP_SERVER_TEST_H
#define HTTP_SERVER_TEST_H

#include "boost/process.hpp"
#include "gtest/gtest.h"

class HttpServerTest : public ::testing::Test
{
  protected:
	HttpServerTest();
	~HttpServerTest();
	boost::process::child m_httpServerProcess;
	boost::process::group m_httpServerGroup;
};

#endif  // HTTP_SERVER_TEST_H
