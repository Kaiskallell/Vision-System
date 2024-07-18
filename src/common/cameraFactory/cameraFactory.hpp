#ifndef VS_CAMERA_FACTORY_HPP
#define VS_CAMERA_FACTORY_HPP
/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * @brief camera factory vision system
 */

#include <experimental/filesystem>

#include "cameraInterface.hpp"
#include "logging.h"

namespace fs = std::experimental::filesystem;

/**
 * @brief it creates a VsCameraInterface object with the help of the config files
 *
 * @param formatConfigPath needed for loading the correct gan for depth estimation
 * @param triggerMode
 * @return std::shared_ptr<VsCameraInterface>
 */
std::shared_ptr<VsCameraInterface> cameraFactory(const size_t camIdx,
                                                 const fs::path& cameraConfigPath,
                                                 const fs::path& formatConfigPath,
                                                 TriggerMode triggerMode,
                                                 bool maxResolution = false);

std::shared_ptr<VsCameraInterface> setupCamera(bool& shutDownReturnVar,
                                               const fs::path& cameraConfigPath,
                                               const fs::path& networksConfigPath,
                                               const std::shared_ptr<spdlog::logger> kLogger);

#endif  // VS_CAMERA_FACTORY_HPP
