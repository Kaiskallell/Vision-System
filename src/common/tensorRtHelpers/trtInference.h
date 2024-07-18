/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef TRT_INF_H
#define TRT_INF_H

#include <experimental/filesystem>
#include <vector>

#include "logging.h"
#include "opencv2/opencv.hpp"

namespace fs = std::experimental::filesystem;

namespace trtInf
{
struct InputData
{
	std::string layername = "";
	cv::Mat img;
	int32_t inputIdx = -1;
	int32_t inputLayerSize = -1;
};

// this class is same as nvinfer1::Dims
// this should break dependency chains
class LayerDim32
{
  public:
	static constexpr int32_t MAX_DIMS{8};
	int32_t nbDims;
	int32_t d[MAX_DIMS];
};

class TrtInference
{
  public:
	TrtInference(const fs::path& onnxPath,
	             const std::vector<std::string>& inputLayerNames,
	             const std::vector<std::string>& outputLayerNames,
	             const std::string& dataType = "fp16",
	             const std::vector<fs::path>& calibImgsFolders = {});
	~TrtInference();
	void run(std::vector<InputData>& blobs, std::vector<std::shared_ptr<float[]>> outputs);
	int32_t getLayerSize(const std::string& layername);
	LayerDim32 getLayerDim(const std::string& layername);

  protected:
	class Impl;
	std::unique_ptr<Impl> m_impl;
	std::vector<std::string> m_inputLayerNames;
	std::vector<fs::path> m_calibImgsFolders;
	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("TrtInference");
};

}  // namespace trtInf

#endif
