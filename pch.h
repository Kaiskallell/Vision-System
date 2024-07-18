// pch.h: This is a precompiled header file.
// Files listed below are compiled only once, improving build performance for future builds.
// This also affects IntelliSense performance, including code completion and many code browsing features.
// However, files listed here are ALL re-compiled if any one of them is updated between builds.
// Do not add files here that you will be updating frequently as this negates the performance advantage.

#ifndef VISION_SYSTEM_PCH_H
#define VISION_SYSTEM_PCH_H

#include <experimental/filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "libs/external/spdlog/include/spdlog/fmt/bundled/ostream.h"
#include "libs/external/spdlog/include/spdlog/sinks/ostream_sink.h"
#include "libs/external/spdlog/include/spdlog/sinks/stdout_color_sinks.h"
#include "libs/external/spdlog/include/spdlog/sinks/stdout_sinks.h"
#include "libs/external/spdlog/include/spdlog/spdlog.h"
#include "libs/json.hpp"

#endif