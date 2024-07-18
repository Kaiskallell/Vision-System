/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef ENTROPY_CALIBRATOR_H
#define ENTROPY_CALIBRATOR_H

#include <experimental/filesystem>

#include "NvInfer.h"

namespace entropycalib
{
namespace fs = std::experimental::filesystem;

class Int8EntropyCalibrator2 : public nvinfer1::IInt8EntropyCalibrator2
{
  public:
	Int8EntropyCalibrator2(const std::string& calibrationTableName,
	                       const std::vector<std::string>& inputBlobNames,
	                       const std::vector<nvinfer1::Dims>& inputDims,
	                       const std::vector<fs::path>& calibImgsFolders,
	                       bool readCache = true);

	~Int8EntropyCalibrator2();

	int getBatchSize() const noexcept override { return 1; }

	bool getBatch(void* bindings[], const char* names[], int nbBindings) noexcept override;

	const void* readCalibrationCache(size_t& length) noexcept override;

	void writeCalibrationCache(const void* cache, size_t length) noexcept override;

  private:
	std::vector<std::string> m_inputBlobNames;  // input layer names
	const bool m_readCache;
	std::vector<void*> m_deviceInputs;
	std::vector<nvinfer1::Dims> m_dims;              // inputDims of network
	std::vector<std::vector<fs::path>> m_calibImgs;  // img folders for different inputs containing imgs
	size_t m_currentImgIdx = 0;
	const std::string m_calibrationTableName;
	std::vector<char> m_calibrationCache;
};
}  // namespace entropycalib
#endif  // DLC_ENTROPY_CALIBRATOR_H