/**
 * @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef VS_MISC_H
#define VS_MISC_H

#include <opencv2/opencv.hpp>

#include "cameraInterface.hpp"
#include "logging.h"
#include "projectPaths.h"
#include "vs_image.hpp"
#include "vs_poseObject.h"

namespace misc
{
void getDetectionAreaPolygon(std::vector<cv::Point>& polygonDetectArea, const fs::path& visionSytemConfigPath);
cv::Mat getDetectionAreaMask(const fs::path& cameraConfigPath,
                             const std::string& serialNumber,
                             const cv::Size& maskSize,
                             const std::shared_ptr<spdlog::logger> logger);
void saveDetectionsToDisk(const std::vector<VSPose>& posesInRobotCoordinates,
                          const VsFrame& frame,
                          const uint64_t frameNumber,
                          const std::shared_ptr<spdlog::logger> logger);

void setVisualization(bool enableFlag);
void waitKey1ms();
void showImage(const std::string& caption, const cv::Mat& src);

void getFrame(VsFrame& frame, cv::Mat& debugResImg, const std::shared_ptr<VsCameraInterface> camera);
void stechtestPoseSelection(size_t& poseIdx, std::vector<VSPose>& posesInRobotCoordinates, std::vector<int>& classIds);
void parseCommandLineInput(int argc,
                           char* argv[],
                           fs::path& cameraConfigPath,
                           fs::path& visionSystemConfigPath,
                           fs::path& formatConfigPath,
                           fs::path& networksConfigPath);

}  // namespace misc

#endif
