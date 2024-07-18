/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef CLASSIFICATIONEXTENSION_TEST_H
#define CLASSIFICATIONEXTENSION_TEST_H

#include <boost/process.hpp>

#include "dbFacade/dbFacade.h"
#include "gtest/gtest.h"
namespace bp = boost::process;

class ClassificationTest : public ::testing::Test
{
  protected:
	ClassificationTest();
	~ClassificationTest();
	bp::child m_classificatorExtensionProcess;
	std::unique_ptr<db::DbFacade> m_dbFacade;
};

#endif  // CLASSIFICATIONEXTENSION_TEST_H
