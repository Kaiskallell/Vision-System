/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef LOGGING_H
#define LOGGING_H

#include "spdlog/fmt/bundled/ostream.h"  // NEEDED by anyone who wants to use spdlog with overloaded operator<<
#include "spdlog/spdlog.h"

namespace logging
{
enum Level
{
	Off = -1,
	Critical = 0,
	Err = 1,
	Warn = 2,
	Info = 3,
	Debug = 4,
	Trace = 5
};

/**
 * @brief create a logger with a given name
 */
std::shared_ptr<spdlog::logger> setupLogger(const std::string& name);

void setDefaultPattern(std::shared_ptr<spdlog::logger>& logger);
}  // namespace logging

#endif  // LOGGING_H