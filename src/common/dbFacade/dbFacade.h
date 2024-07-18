/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#ifndef DB_FACADE_H
#define DB_FACADE_H

#include <experimental/filesystem>
#include <memory>

#include "Format.pb.h"
#include "logging.h"
#include "redis++.h"
#include "vs_poseObject.h"

namespace fs = std::experimental::filesystem;
namespace db
{
struct DbConfig
{
	std::string databaseLink = "tcp://127.0.0.1:6379";
	size_t maxProductsInDb = 2048;
};

struct ImprintMap
{
	std::string imprintName;
	uint32_t classId;
	uint32_t productId;
};

typedef google::protobuf::RepeatedPtrField<VMS::VisionSystem::ImprintMapping> ImprintMappings;

enum class ObjectType : int
{
	product = 0,
	placeTray = 1
};

class DbFacade
{
  public:
	explicit DbFacade(const fs::path& dbConfigPath);
	~DbFacade() = default;
	void flushDB();
	std::shared_ptr<DbConfig> readConfig(const fs::path& configFilePath) const;
	/**
	 * @brief It serializes the poses and classes to a protobuf object and writes that string to database
	 */
	void serializeProductInfosToDB(const std::vector<VSPose>& posesInRobotCoordinates,
	                               const std::vector<int>& classIds,
	                               const int32_t rotEncoderValue,
	                               const uint64_t frameId,
	                               const uint32_t areaNumber,
	                               const ObjectType objType);
	void serializeTrayInfosToDB(const std::vector<VSPose>& posesInRobotCoordinates,
	                            const std::vector<int>& classIds,
	                            const int32_t rotEncoderValue,
	                            const uint64_t frameId,
	                            const std::vector<uint64_t>& trackingIds,
	                            const uint32_t areaNumber);
	std::string getSerializedObjInfosFromDB(const int areaNumber);
	void deleteSerializedObjInfosFromDB();
	bool checkIfFormatChanged();
	void setFormatChanged(bool workChainChanged);
	void writeFormatToDB(const uint32_t formatId);
	/**
	 * @brief imprintMappings convert from classID(output of network) to productId(needed by VMS)
	 */
	void writeImprintMappingToDB(const ImprintMappings imprintMappings);
	std::vector<ImprintMap> getImprintMappingFromDB();
	uint32_t getFormatId();
	uint64_t getFrameCounter(size_t areaNumber);
	void writeFrameCounterToDb(const uint64_t frameCounter, const size_t areaNumber);
	void setShutDownProcesses(bool shutdown);
	bool getShutDownProcesses();
	bool getExtReq(const int areaNumber);               // for extensions like classification
	void setExtReq(bool extReq, const int areaNumber);  // for extensions like classification
	void writeErrorCodeToDB(const int errorCode);
	int getErrorCodeFromDB();
  private:
	std::shared_ptr<sw::redis::Redis> m_redis;
	std::shared_ptr<DbConfig> m_config;
	uint64_t m_objectUid = 0;
	const size_t m_kVisionSystemId = 1;

	// database keys
	std::string m_frameCounterKey1 = "";
	std::string m_frameCounterKey2 = "";
	std::string m_frameCounterKey3 = "";
	std::string m_frameCounterKey4 = "";
	std::string m_frameCounterKey7 = "";
	std::string m_frameCounterKey8 = "";

	std::string m_workChainChangedPickKey = "";
	std::string m_formatKey = "";
	std::string m_classIdMappingKey = "";
	std::string m_productIdMappingKey = "";
	std::string m_imprintNameMappingKey = "";
	// std::string m_sceneInfosKey = "";
	std::string m_sceneInfosKeyArea1 = "";
	std::string m_sceneInfosKeyArea2 = "";
	std::string m_sceneInfosKeyArea3 = "";
	std::string m_sceneInfosKeyArea4 = "";
	std::string m_sceneInfosKeyArea7 = "";
	std::string m_sceneInfosKeyArea8 = "";

	std::string m_extReqKey7 = "";
	std::string m_extReqKey8 = "";

	std::string m_shutDownProcesses = "";
	std::string m_errorCodeKey = "";
	inline static const std::shared_ptr<spdlog::logger> m_kLogger = logging::setupLogger("DbFacade");
};

uint32_t checkIfClassIdsAreInImprintMapping(const std::vector<int>& classIds,
                                            std::vector<db::ImprintMap>& imprintMappingTable,
                                            const std::shared_ptr<spdlog::logger> logger);

void writePosesToDB(const std::shared_ptr<db::DbFacade> dbFacade,
                    const std::vector<VSPose>& posesInRobotCoordinates,
                    const std::vector<int>& classIds,
                    const uint64_t frameNumber,
                    const uint32_t rotEncoderValue,
                    const size_t areaNumber,
                    const std::vector<db::ImprintMap>& imprintMappingTable,
                    const std::shared_ptr<spdlog::logger> m_kLogger);

void setupDBObjectForDetection(std::shared_ptr<db::DbFacade> dbFacade,
                               std::vector<db::ImprintMap>& imprintMappingTable,
                               const fs::path& visionSystemConfigPath,
                               const int formatId,
                               const size_t areaNumber,
                               const std::shared_ptr<spdlog::logger> kLogger);

}  // namespace db

#endif
