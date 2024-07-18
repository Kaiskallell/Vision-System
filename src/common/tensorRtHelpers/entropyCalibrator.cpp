/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "entropyCalibrator.h"

#include <cuda_runtime_api.h>

#include <fstream>   // ofstream
#include <iterator>  // istream_iterator
#include <opencv2/opencv.hpp>

namespace entropycalib
{
#ifndef CUDA_CHECK
#define CUDA_CHECK(callstr)                                                                    \
	{                                                                                          \
		cudaError_t error_code = callstr;                                                      \
		if (error_code != cudaSuccess)                                                         \
		{                                                                                      \
			std::cerr << "CUDA error " << error_code << " at " << __FILE__ << ":" << __LINE__; \
			assert(0);                                                                         \
		}                                                                                      \
	}
#endif  // CUDA_CHECK

int32_t getLayerSize(const nvinfer1::Dims& dims)
{
	int32_t layerSize = 1;
	for (int j = 0; j < dims.nbDims; ++j)
	{
		layerSize *= dims.d[j];
	}
	return layerSize;
}

Int8EntropyCalibrator2::Int8EntropyCalibrator2(const std::string& calibrationTableName,
                                               const std::vector<std::string>& inputBlobNames,
                                               const std::vector<nvinfer1::Dims>& inputDims,
                                               const std::vector<fs::path>& calibImgsFolders,
                                               bool readCache)
    : m_calibrationTableName(calibrationTableName),
      m_inputBlobNames(inputBlobNames),
      m_dims(inputDims),
      m_readCache(readCache)
{
	// read all the path of the imgs which are in the calibImgs folder
	m_calibImgs.reserve(calibImgsFolders.size());
	for (size_t i = 0; i < calibImgsFolders.size(); ++i)
	{
		m_calibImgs.emplace_back();
		const fs::directory_iterator end{};
		for (fs::directory_iterator iter{calibImgsFolders.at(i)}; iter != end; ++iter)
		{
			if (fs::is_regular_file(*iter))
			{
				m_calibImgs.at(i).push_back(iter->path());
			}
		}
	}

	m_deviceInputs.reserve(m_dims.size());
	for (size_t i = 0; i < m_dims.size(); ++i)
	{
		m_deviceInputs.emplace_back();
		size_t imgSize = getLayerSize(m_dims.at(i));
		CUDA_CHECK(cudaMalloc(&m_deviceInputs.at(i), imgSize * sizeof(float)));
	}
}

Int8EntropyCalibrator2::~Int8EntropyCalibrator2()
{
	for (size_t i = 0; m_dims.size(); ++i)
	{
		CUDA_CHECK(cudaFree(m_deviceInputs.at(i)));
	}
}

bool Int8EntropyCalibrator2::getBatch(void* bindings[], const char* names[], int nbBindings) noexcept
{
	for (size_t cameraIdx = 0; cameraIdx < m_calibImgs.size(); ++cameraIdx)
	{
		if (m_currentImgIdx >= m_calibImgs.at(cameraIdx).size())
		{
			return false;  // end of calibration, no more batches
		}

		cv::Mat img;
		try
		{
			img = cv::imread(m_calibImgs.at(cameraIdx).at(m_currentImgIdx));
		}
		catch (const std::exception& e)
		{
			throw std::runtime_error("cannot read img for calibration: "
			                         + m_calibImgs.at(cameraIdx).at(m_currentImgIdx).string());
		}

		assert(!img.empty());

		// preprocessing needs to be done beforehand
		bool swapRB = true;
		bool crop = true;
		float scaling = 1.0 / 255.0;
		cv::Scalar mean = cv::Scalar(0, 0, 0);
		cv::Mat blob;
		cv::dnn::blobFromImage(img, blob, scaling, cv::Size(img.cols, img.rows), mean, swapRB, crop);

		size_t imgSize = img.cols * img.rows * img.channels();
		CUDA_CHECK(
		    cudaMemcpy(m_deviceInputs.at(cameraIdx), blob.data, imgSize * sizeof(float), cudaMemcpyHostToDevice));
		// assert(!strcmp(names[0], m_inputBlobNames.at(0)));
		bindings[cameraIdx] = m_deviceInputs.at(cameraIdx);
	}
	m_currentImgIdx++;
	return true;
}

const void* Int8EntropyCalibrator2::readCalibrationCache(size_t& length) noexcept
{
	m_calibrationCache.clear();
	std::ifstream input(m_calibrationTableName, std::ios::binary);
	input >> std::noskipws;
	if (m_readCache && input.good())
	{
		std::copy(
		    std::istream_iterator<char>(input), std::istream_iterator<char>(), std::back_inserter(m_calibrationCache));
	}
	length = m_calibrationCache.size();
	return length ? m_calibrationCache.data() : nullptr;
}

void Int8EntropyCalibrator2::writeCalibrationCache(const void* cache, size_t length) noexcept
{
	std::ofstream output(m_calibrationTableName, std::ios::binary);
	output.write(reinterpret_cast<const char*>(cache), length);
}
}  // namespace entropycalib