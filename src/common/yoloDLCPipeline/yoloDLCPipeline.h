/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef YOLO_DLC_PIPELINE_H
#define YOLO_DLC_PIPELINE_H

#include <experimental/filesystem>
#include <memory>
#include <opencv2/opencv.hpp>

#include "logging.h"

namespace fs = std::experimental::filesystem;

struct YoloDLCResults
{
	std::vector<cv::Rect> m_rects;
	std::vector<std::vector<cv::Point2f>> m_keyPoints;
	std::vector<int> m_classIds;
};

class YoloDLCPipeline
{
  public:
	explicit YoloDLCPipeline(const fs::path& networksPath);
	~YoloDLCPipeline();
	bool m_moduleConfigured = false;
	YoloDLCResults run(cv::Mat& src, const size_t nMaxResults, const int desiredClass);

  private:
	class Impl;
	std::unique_ptr<Impl> m_impl;  // pointer to implementation to reduce dependency chains
	std::map<int, int> m_classToDlcIdxMapping;
	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("yoloDLCPipeline");
};

namespace YoloDLC
{
constexpr int kAllClasses = -1;
std::shared_ptr<YoloDLCPipeline> setupYoloDlc(const fs::path& networksConfigPath,
                                              const std::shared_ptr<spdlog::logger> kLogger);

}  // namespace YoloDLC

#endif
