/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "logging.h"

#include "spdlog/sinks/ostream_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

namespace logging
{
std::shared_ptr<spdlog::logger> setupLogger(const std::string& name)
{
	std::vector<spdlog::sink_ptr> sinks;
	auto colorStdoutSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
	colorStdoutSink->set_level(spdlog::level::trace);
	sinks.push_back(colorStdoutSink);

	/*
	 * Setting two sinks: one for stout and one for stderr causes duplicate messages
	 * on the console for Error and Critical types, since both of these will be printed
	 * both on stdout and stderr.
	 * For the time being, disabling the stderr sink.
	 * Alternatively, one can implement a custom sink as proposed here
	 * https://github.com/gabime/spdlog/issues/345#issuecomment-271117409
	 * which will pipe the messages on appropriate output depending on level
	 */

	// auto colorStderrSink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
	// colorStderrSink->set_level(spdlog::level::err);
	// sinks.push_back(colorStderrSink);

	std::string logFile = "vsLogs.log";
	size_t fileSize = 1024 * 1024 * 5;
	size_t maxNLogFiles = 3;
	auto fileSink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(logFile, fileSize, maxNLogFiles, true);
	fileSink->set_level(spdlog::level::trace);
	sinks.push_back(fileSink);

	auto loggerTest = spdlog::get(name);

	std::string newName = name;
	if (loggerTest != nullptr){
		newName = name + "1";
	}
	auto logger = std::make_shared<spdlog::logger>(name, std::begin(sinks), std::end(sinks));

	setDefaultPattern(logger);
	// overrides previous level settings
	logger->set_level(spdlog::level::debug);
	spdlog::register_logger(logger);

	return logger;
}

void setDefaultPattern(std::shared_ptr<spdlog::logger>& logger)
{
	// logger->set_pattern("[%H:%M:%S %z] [%=12n] [%^--%L--%$] %v");
	logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%=12n] [%^--%L--%$] %v");
}

}  // namespace logging
