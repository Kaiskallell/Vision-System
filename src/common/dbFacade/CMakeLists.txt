
add_library(dbFacade 
	dbFacade.cpp
	${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp
)

target_include_directories(dbFacade  
	PRIVATE 
	${CMAKE_SOURCE_DIR}/libs #json
	PUBLIC
	${CMAKE_SOURCE_DIR}/src/common
	${CMAKE_SOURCE_DIR}/src/common/logging
	${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(dbFacade 
	PRIVATE
	spdlog
	PUBLIC
	proto_messages_lib
	RedisClient
)
