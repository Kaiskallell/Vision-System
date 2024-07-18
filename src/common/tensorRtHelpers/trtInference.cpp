/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "trtInference.h"

#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <NvOnnxParser.h>

#include <experimental/filesystem>
#include <fstream>

#include "cuda_utils.h"
#include "entropyCalibrator.h"
#include "trtLogging.h"

#define DEVICE_NR 0  // GPU id

namespace trtInf
{
namespace fs = std::experimental::filesystem;

constexpr long long int operator"" _MiB(unsigned long long val) { return val * (1 << 20); }

typedef std::vector<nvinfer1::Dims> IODimensions;

class TrtInference::Impl
{
  public:
	Impl() = default;
	~Impl() = default;
	void doInference(nvinfer1::IExecutionContext& context,
	                 cudaStream_t& stream,
	                 void** buffers,
	                 std::vector<std::shared_ptr<float[]>> outputs,
	                 int batchSize,
	                 const std::vector<int>& outputIdxs) const;
	std::string openEngineFile(const fs::path& enginePath);
	fs::path createEngineAndWriteToDisk(fs::path onnxPath,
	                                    std::shared_ptr<nvinfer1::INetworkDefinition> network,
	                                    std::shared_ptr<nvinfer1::IBuilder> builder,
	                                    std::shared_ptr<trtLog::Logger> trtLogger,
	                                    const std::vector<std::string>& inputLayerNames,
	                                    const std::vector<fs::path>& calibImgsFolders,
	                                    const nvinfer1::BuilderFlag& inferDataType);
	IODimensions m_allBindingDims;
	std::vector<int> m_outputIdxs;
	std::shared_ptr<nvinfer1::ICudaEngine> m_engine;
	std::unique_ptr<nvinfer1::IExecutionContext>
	    m_context;  // context needs to be declared after engine, because otherwise a segfault will happen in destructor
	                // this way m_context will be destroyed first and then m_engine (members are destroyed in reverse
	                // order)
	cudaStream_t m_stream;
	float** m_iOBuffers;
	float** m_imgHostPinned;
	int m_batchsize = -1;
};

IODimensions getInputDims(std::shared_ptr<nvinfer1::INetworkDefinition> network)
{
	int32_t nbInputs = network->getNbInputs();
	IODimensions dims;
	for (int32_t i = 0; i < nbInputs; ++i)
	{
		dims.emplace_back(network->getInput(i)->getDimensions());
	}
	return dims;
}

size_t getLayerIdx(std::shared_ptr<nvinfer1::INetworkDefinition> network, const std::string& layerName)
{
	for (int idx = 0; idx < network->getNbLayers(); ++idx)
	{
		std::string netLayerName = network->getLayer(idx)->getName();
		if (netLayerName == layerName)
		{
			return idx;
		}
	}
	throw std::runtime_error("specified layer does not exist in network: " + layerName);
}

void setPrecisionOfLayersToFp32(std::shared_ptr<nvinfer1::INetworkDefinition> network,
                                const std::vector<std::string>& layerNames)
{
	for (const auto& layerName : layerNames)
	{
		size_t idx = getLayerIdx(network, layerName);
		std::cout << "idx is = " << idx << std::endl;
		network->getLayer(idx)->setPrecision(nvinfer1::DataType::kFLOAT);  // kFLOAT is 32-bit float
	}
}

void switchLayerwiseToFp32(std::shared_ptr<nvinfer1::INetworkDefinition> network,
                           nvinfer1::IBuilderConfig* builderConfig)
{
	std::vector<std::string> layerNames = {"Conv_177",
	                                       "Conv_182",
	                                       "Conv_187",
	                                       "Conv_194",
	                                       "Conv_199",
	                                       "Conv_216",
	                                       "Conv_206",
	                                       "Conv_211",
	                                       "Conv_222",
	                                       "Conv_227",
	                                       "Conv_244",
	                                       "Conv_234",
	                                       "Conv_239",
	                                       "Conv_250",
	                                       "Conv_255",
	                                       "Conv_262"};
	builderConfig->setFlag(nvinfer1::BuilderFlag::kOBEY_PRECISION_CONSTRAINTS);
	setPrecisionOfLayersToFp32(network, layerNames);
}

IODimensions getDims(std::shared_ptr<nvinfer1::ICudaEngine> engine)
{
	int32_t nbBindings = engine->getNbBindings();
	IODimensions dims;
	for (int32_t i = 0; i < nbBindings; ++i)
	{
		dims.emplace_back(engine->getBindingDimensions(i));
	}
	return dims;
}

size_t getNbInputBindings(std::shared_ptr<nvinfer1::ICudaEngine> engine)
{
	int32_t nbBindings = engine->getNbBindings();
	size_t nbInutBindings = 0;
	for (int32_t i = 0; i < nbBindings; ++i)
	{
		if (engine->bindingIsInput(i))
		{
			++nbInutBindings;
		}
	}
	return nbInutBindings;
}

int32_t getLayerSize(const nvinfer1::Dims& dims)
{
	int32_t layerSize = 1;
	for (int j = 0; j < dims.nbDims; ++j)
	{
		layerSize *= dims.d[j];
	}
	return layerSize;
}

int32_t getLayerSize(const std::shared_ptr<nvinfer1::ICudaEngine> engine, const size_t idx)
{
	int32_t layerSize = 1;
	if (engine->getNbBindings() - 1 < idx)
	{
		throw std::runtime_error("TrtInference: engine binding idx is greater than engine->getNbBindings()="
		                         + std::to_string(engine->getNbBindings()));
	}
	for (int j = 0; j < engine->getBindingDimensions(idx).nbDims; ++j)
	{
		layerSize *= engine->getBindingDimensions(idx).d[j];
	}
	return layerSize;
}

int32_t TrtInference::getLayerSize(const std::string& layername)
{
	int32_t layerIdx = m_impl->m_engine->getBindingIndex(layername.c_str());
	if (layerIdx == -1)
	{
		throw std::runtime_error("TrtInference: layername not found: " + layername);
	}
	int32_t layerSize = trtInf::getLayerSize(m_impl->m_engine, layerIdx);
	return layerSize;
}

LayerDim32 TrtInference::getLayerDim(const std::string& layername)
{
	int32_t layerIdx = m_impl->m_engine->getBindingIndex(layername.c_str());
	if (layerIdx == -1)
	{
		throw std::runtime_error("TrtInference: layername not found: " + layername);
	}

	nvinfer1::Dims dim = m_impl->m_engine->getBindingDimensions(layerIdx);
	LayerDim32 layerDim;
	layerDim.nbDims = dim.nbDims;
	for (int i = 0; i < layerDim.MAX_DIMS; ++i)
	{
		layerDim.d[i] = dim.d[i];
	}
	return layerDim;
}

fs::path makeEnginePathWithInferDataType(const fs::path& onnxPath, const nvinfer1::BuilderFlag& inferDataType)
{
	fs::path enginePath = onnxPath;
	std::string engineSuffix = "";
	if (inferDataType == nvinfer1::BuilderFlag::kINT8)
	{
		engineSuffix = ".int8.engine";
	}
	else if (inferDataType == nvinfer1::BuilderFlag::kFP16)
	{
		engineSuffix = ".fp16.engine";
	}
	else if (inferDataType == nvinfer1::BuilderFlag::kTF32)
	{
		engineSuffix = ".fp32.engine";
	}
	else
	{
		throw std::runtime_error("makeEnginePathWithInferDataType(): inferDataType not known");
	}
	enginePath.replace_extension(engineSuffix);
	return enginePath;
}

bool checkIfEngineAlreadyExists(const fs::path& enginePath, std::shared_ptr<trtLog::Logger> trtLogger)
{
	if (fs::is_regular_file(enginePath) && fs::file_size(enginePath) != 0)
	{
		std::cout << "reusing already created .engine file: {}" << enginePath.string();
		bool ret = initLibNvInferPlugins(trtLogger.get(),
		                                 "");  // needed because parser is not called which would init it usually
		return true;
	}
	return false;
}

fs::path TrtInference::Impl::createEngineAndWriteToDisk(fs::path onnxPath,
                                                        std::shared_ptr<nvinfer1::INetworkDefinition> network,
                                                        std::shared_ptr<nvinfer1::IBuilder> builder,
                                                        std::shared_ptr<trtLog::Logger> trtLogger,
                                                        const std::vector<std::string>& inputLayerNames,
                                                        const std::vector<fs::path>& calibImgsFolders,
                                                        const nvinfer1::BuilderFlag& inferDataType)
{
	std::cout << "onnxPath = {}" << onnxPath << std::endl;
	if (!fs::is_regular_file(onnxPath) || onnxPath.empty())
	{
		throw std::runtime_error("TrtInference: onnxPath does not exist or is empty!");
	}
	if (onnxPath.extension() != ".onnx")
	{
		throw std::runtime_error("TrtInference: onnxPath does not have a '.onnx' extension!");
	}

	if (inferDataType == nvinfer1::BuilderFlag::kINT8 && calibImgsFolders.at(0).empty())
	{
		throw std::runtime_error("int8 optimization choosen but no calibPaths specified");
	}

	fs::path enginePath = makeEnginePathWithInferDataType(onnxPath, inferDataType);
	if (checkIfEngineAlreadyExists(enginePath, trtLogger))
	{
		return enginePath;
	}

	// parser and network are coupled. If parser gets destroyed network weights
	// will get invalid
	std::shared_ptr<nvonnxparser::IParser> parser{nvonnxparser::createParser(*network, *trtLogger)};
	// parse the onnx file, parser is owner of the weights
	std::cout << "starting to parse onnx:  " << onnxPath << std::endl;
	bool ret = parser->parseFromFile(onnxPath.c_str(), (int)nvinfer1::ILogger::Severity::kVERBOSE);
	if (!ret)
	{
		throw std::runtime_error("TrtInference: Could not parse network from onnx file: " + onnxPath.string());
	}
	std::cout << "finished parsing onnx " << std::endl;

	nvinfer1::IBuilderConfig* builderConfig = builder->createBuilderConfig();

	constexpr int kMagicMaxBatchSize = 5;
	builder->setMaxBatchSize(kMagicMaxBatchSize);
	builderConfig->setMaxWorkspaceSize(4024_MiB);

	builderConfig->setFlag(inferDataType);
	std::unique_ptr<entropycalib::Int8EntropyCalibrator2> calibrator;
	if (inferDataType == nvinfer1::BuilderFlag::kINT8)
	{
		std::cout << "using  int8 optimization" << std::endl;
		assert(builder->platformHasFastInt8());
		builderConfig->setFlag(nvinfer1::BuilderFlag::kINT8);
		std::string calibrationTableName = enginePath.stem().string() + ".calibration.table";
		std::vector<nvinfer1::Dims> inputDims = getInputDims(network);
		calibrator = std::make_unique<entropycalib::Int8EntropyCalibrator2>(
		    calibrationTableName, inputLayerNames, inputDims, calibImgsFolders);
		builderConfig->setInt8Calibrator(calibrator.get());
	}

	// switchLayerwiseToFp32(network, builderConfig);

	m_kLogger->debug("building engine");
	std::shared_ptr<nvinfer1::IHostMemory> serializedModel{builder->buildSerializedNetwork(*network, *builderConfig)};

	assert(serializedModel != nullptr);
	m_kLogger->debug("enginePath = {}", enginePath);
	std::ofstream p(enginePath, std::ios::binary);
	if (!p)
	{
		throw std::runtime_error("TrtInference: cannot write engine to file!");
	}
	p.write(reinterpret_cast<const char*>(serializedModel->data()), serializedModel->size());

	return enginePath;
}

std::string TrtInference::Impl::openEngineFile(const fs::path& enginePath)
{
	std::ifstream file(enginePath, std::ios::binary);
	if (!file.good())
	{
		throw std::runtime_error("TrtInference: cannot read engine file! enginePath = " + enginePath.string());
	}

	std::stringstream buffer;
	buffer << file.rdbuf();
	std::string trtModel = buffer.str();

	return std::move(trtModel);
}

void TrtInference::Impl::doInference(nvinfer1::IExecutionContext& context,
                                     cudaStream_t& stream,
                                     void** buffers,
                                     std::vector<std::shared_ptr<float[]>> outputs,
                                     int batchSize,
                                     const std::vector<int>& outputIdxs) const
{
	// infer on the batch asynchronously, and DMA output back to host
	context.enqueueV2(buffers, stream, nullptr);

	if (outputs.size() != outputIdxs.size())
	{
		throw std::runtime_error("TrtInference: outputs and outputIdxs are not the same size");
	}

	for (size_t i = 0; i < outputIdxs.size(); ++i)
	{
		int32_t outputLayerSize = trtInf::getLayerSize(m_engine, outputIdxs.at(i));
		// every output must be copied back to cpu
		CUDA_CHECK(cudaMemcpyAsync(outputs.at(i).get(),
		                           buffers[outputIdxs.at(i)],
		                           outputLayerSize * sizeof(float),
		                           cudaMemcpyDeviceToHost,
		                           stream));
	}

	cudaStreamSynchronize(stream);
}

TrtInference::TrtInference(const fs::path& onnxPath,
                           const std::vector<std::string>& inputLayerNames,
                           const std::vector<std::string>& outputLayerNames,
                           const std::string& dataType,
                           const std::vector<fs::path>& calibImgsFolders)
    : m_inputLayerNames(inputLayerNames), m_calibImgsFolders(calibImgsFolders)
{
	cudaSetDevice(DEVICE_NR);
	m_impl = std::make_unique<Impl>();
	std::shared_ptr<trtLog::Logger> trtLogger = trtLog::getGlobalLogger();
	std::unique_ptr<nvinfer1::IRuntime> runtime{nvinfer1::createInferRuntime(*trtLogger)};
	assert(runtime != nullptr);

	// Create builder
	std::shared_ptr<nvinfer1::IBuilder> builder{nvinfer1::createInferBuilder(*trtLogger)};
	// explicit batch is needed for onnx import
	uint32_t flag = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
	std::shared_ptr<nvinfer1::INetworkDefinition> network{builder->createNetworkV2(flag)};

	nvinfer1::BuilderFlag inferDataType = nvinfer1::BuilderFlag::kFP16;
	if (dataType == "int8")
	{
		inferDataType = nvinfer1::BuilderFlag::kINT8;
	}
	if (dataType == "fp32")
	{
		inferDataType = nvinfer1::BuilderFlag::kTF32;
	}
	fs::path enginePath = m_impl->createEngineAndWriteToDisk(
	    onnxPath, network, builder, trtLogger, m_inputLayerNames, m_calibImgsFolders, inferDataType);

	// deserialize the .engine and run inference
	std::string trtModel = m_impl->openEngineFile(enginePath);
	m_impl->m_engine =
	    std::shared_ptr<nvinfer1::ICudaEngine>(runtime->deserializeCudaEngine(trtModel.c_str(), trtModel.size()));
	assert(m_impl->m_engine != nullptr);
	m_impl->m_context = std::unique_ptr<nvinfer1::IExecutionContext>(m_impl->m_engine->createExecutionContext());
	assert(m_impl->m_context != nullptr);

	m_impl->m_allBindingDims = getDims(m_impl->m_engine);

	assert(m_impl->m_allBindingDims.at(0).nbDims == 4);  // e.g. 1x3x128x128
	m_impl->m_batchsize = m_impl->m_allBindingDims.at(0).d[0];

	if (m_impl->m_allBindingDims.size() < inputLayerNames.size() + outputLayerNames.size())
	{
		throw std::runtime_error("TrtInference: m_Dims.size() > inputLayerNames.size() + outputLayerNames.size() "
		                         "Specify all IO layer names.");
	}

	// In order to bind the buffers, we need to know the names of the input and
	// output tensors. Note that indices are guaranteed to be less than
	// IEngine::getNbBindings()

	// Create GPU buffers on device for input
	typedef float* FloatPtr;
	m_impl->m_iOBuffers = new FloatPtr[m_impl->m_allBindingDims.size()];
	m_impl->m_imgHostPinned = new FloatPtr[getNbInputBindings(m_impl->m_engine)];

	for (size_t i = 0; i < m_impl->m_allBindingDims.size(); ++i)
	{
		int32_t layerSize = trtInf::getLayerSize(m_impl->m_allBindingDims.at(i));
		CUDA_CHECK(cudaMalloc((void**)&m_impl->m_iOBuffers[i], layerSize * sizeof(float)));
		// prepare input data cache in pinned memory
		if (m_impl->m_engine->bindingIsInput(i))
		{
			CUDA_CHECK(cudaMallocHost((void**)&m_impl->m_imgHostPinned[i], layerSize * sizeof(float)));
		}
	}

	// Create GPU buffers on device for output
	for (size_t i = 0; i < outputLayerNames.size(); ++i)
	{
		int32_t outputIdx = m_impl->m_engine->getBindingIndex(outputLayerNames.at(i).c_str());
		m_impl->m_outputIdxs.emplace_back(outputIdx);
	}

	// Create stream
	CUDA_CHECK(cudaStreamCreate(&m_impl->m_stream));
}

TrtInference::~TrtInference()
{
	m_kLogger->debug("TrtInference destructor");
	// Release stream and buffers
	cudaStreamDestroy(m_impl->m_stream);

	for (int i = 0; i < m_impl->m_allBindingDims.size(); ++i)
	{
		if (m_impl->m_iOBuffers[i] != nullptr)
		{
			CUDA_CHECK(cudaFree(m_impl->m_iOBuffers[i]));
		}

		if (m_impl->m_engine->bindingIsInput(i))
		{
			if (m_impl->m_imgHostPinned[i] != nullptr)
			{
				CUDA_CHECK(cudaFreeHost(m_impl->m_imgHostPinned[i]));
			}
		}
	}
	delete[] m_impl->m_iOBuffers;
	delete[] m_impl->m_imgHostPinned;
}

void TrtInference::run(std::vector<InputData>& blobs, std::vector<std::shared_ptr<float[]>> outputs)
{
	double dNetwork = (double)cv::getTickCount();
	for (int i = 0; i < blobs.size(); ++i)
	{
		if (blobs.at(i).img.empty())
		{
			throw std::runtime_error("src image is empty!");
		}
		size_t sizeImage = blobs.at(i).inputLayerSize * sizeof(float);
		// copy data to pinned memory
		memcpy(m_impl->m_imgHostPinned[i], blobs.at(i).img.data, sizeImage);
		// copy data to device memory
		CUDA_CHECK(cudaMemcpyAsync(m_impl->m_iOBuffers[blobs.at(i).inputIdx],
		                           m_impl->m_imgHostPinned[blobs.at(i).inputIdx],
		                           sizeImage,
		                           cudaMemcpyHostToDevice,
		                           m_impl->m_stream));
	}

	// Run inference
	double dNetworkInference = (double)cv::getTickCount();
	m_impl->doInference(*(m_impl->m_context),
	                    m_impl->m_stream,
	                    (void**)m_impl->m_iOBuffers,
	                    outputs,
	                    m_impl->m_batchsize,
	                    m_impl->m_outputIdxs);
	// m_kLogger->debug("doInference  step took ms = {}",
	//                 (((double)cv::getTickCount() - dNetworkInference) / cv::getTickFrequency() * 1000));
}

}  // namespace trtInf
