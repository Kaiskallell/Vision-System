/**
 * @copyright Copyright (c) 2020 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 *
 * @brief format class
 *
 */
#include <experimental/filesystem>
#include <fstream>

// protobuf to json
#include <google/protobuf/util/json_util.h>

#include "projectPaths.h"
#include "vs_c_format.h"

// protofiles
#include "Format.pb.h"
#include "json.hpp"

Format::Format()
{
	m_dbFacade = std::make_unique<db::DbFacade>(utils::getProjectRootDir() / "config/visionSystemConfig.json");
}

void Format::writeFormatIdToDisk(const uint32_t formatId)
{
	// reading formatId from config file
	fs::path formatConfigFilePath = utils::getProjectRootDir() / "format/format.json";
	std::ifstream ifs(formatConfigFilePath.string());
	if (!ifs.good())
	{
		throw std::runtime_error("cannot open config file! formatConfigFilePath = " + formatConfigFilePath.string());
	}
	nlohmann::json jsonFile = nlohmann::json::parse(ifs);
	jsonFile.at("id") = formatId;  // replaceing the old value with the new one
	ifs.close();

	// writing to file
	std::ofstream file(formatConfigFilePath.string());  // overwriting file content
	file << jsonFile;
}

/**
 * BE wants to know the current active format
 *
 * @param[in] sMessageBuffer = received message
 * @param[out] result = send message
 *
 * @return: true = good
 *          false = error ,
 *
 */
bool Format::processFormatLoaded(const std::string& sMessageBuffer, std::string& result)
{
	uint32_t formatId = 0;
	try
	{
		formatId = m_dbFacade->getFormatId();
	}
	catch (const std::exception& e)
	{
		m_kLogger->error(e.what());
		m_kLogger->error("Cannot tell VMS which format is loaded!");
		return false;
	}

	VMS::VisionSystem::FormatLoadedResponse formatLoadedResponse;
	formatLoadedResponse.set_id(formatId);

	setMessageSize(formatLoadedResponse.ByteSize());
	formatLoadedResponse.SerializeToString(&result);
	return true;
}

/**
 * BE wants to know the all installed formats
 *
 * @param[in] sMessageBuffer = received message
 * @param[out] result = send message
 *
 * @return: true = good
 *          false = error
 */
bool Format::processFormatAvailable(const std::string& sMessageBuffer, std::string& result)
{
	VMS::VisionSystem::FormatAvailableResponse formatAvailableResponse;
	fs::path formatDir = utils::getProjectRootDir() / "format";

	if (!fs::exists(formatDir))
	{
		m_kLogger->error("Cannot tell VMS which formats are stored on Jetson because format directory was not found");
		return false;
	}

	for (const auto& entry : fs::directory_iterator(formatDir))
	{
		if (!fs::is_directory(entry))
		{
			continue;  // we are only interested in available format folders
		}

		std::string folderName = entry.path().stem().string();
		formatAvailableResponse.add_format_list(folderName);
	}

	setMessageSize(formatAvailableResponse.ByteSize());
	formatAvailableResponse.SerializeToString(&result);
	return true;
}

/**
 * VMS sends new format request which is processed here
 *
 * @param[in] sMessageBuffer = received message
 * @param[out] result = send message
 *
 */
int Format::processFormat(const std::string& sMessageBuffer, std::string& result)
{
	m_kLogger->debug("new format is requested");
	VMS::VisionSystem::FormatRequest formatRequest;
	// can not extract/parse message
	if (!formatRequest.ParseFromString(sMessageBuffer))
	{
		return FormatParsing;
	}

	google::protobuf::util::JsonPrintOptions JsonOptions;
	JsonOptions.add_whitespace = true;
	JsonOptions.always_print_primitive_fields = true;
	// JsonOptions.always_print_enums_as_ints = true;

	bool requestedFormatFound = false;
	fs::path formatDir = utils::getProjectRootDir() / "format";
	for (const auto& entry : fs::directory_iterator(formatDir))
	{
		if (!fs::is_directory(entry))
		{
			continue;  // we are only interested in available format folders
		}

		int storedFormat = std::stoi(entry.path().stem());
		if (storedFormat == formatRequest.id())
		{
			requestedFormatFound = true;
			break;
		}
	}

	if (!requestedFormatFound)
	{
		m_kLogger->error("Requested Format could not be found: {}", formatRequest.id());
		return FormatDirNotFound;
	}

	m_dbFacade->writeFormatToDB(formatRequest.id());
	writeFormatIdToDisk(formatRequest.id());
	if (formatRequest.imprint_mappings().empty())
	{
		m_kLogger->warn("formatRequest.imprint_mappings() is empty");
	}
	for (size_t i = 0; i < formatRequest.imprint_mappings().size(); ++i)
	{
		uint32_t classId = formatRequest.imprint_mappings().Get(i).class_id();
		uint32_t productId = formatRequest.imprint_mappings().Get(i).product_id();
		std::string imprintName = formatRequest.imprint_mappings().Get(i).imprint_name();
		m_kLogger->debug("formatRequest.imprint_mappings() = {} : {} : {} ", imprintName, classId, productId);
	}
	setMessageSize(0);
	m_dbFacade->writeImprintMappingToDB(formatRequest.imprint_mappings());
	m_dbFacade->setFormatChanged(true);  // needed by missionController
	m_kLogger->debug("New Format is = {}", formatRequest.id());
	// TODO(aschaefer): wait until loading of networks is done (constructor of detectPickable)
	return FormatOk;
}
