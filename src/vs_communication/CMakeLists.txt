find_package(Protobuf REQUIRED)

add_subdirectory(proto)

set(VS_COMMUNICATION_SERVER_SOURCES 
	main.cpp
	vs_c_tcpAcceptor.cpp
	vs_c_tcpStream.cpp
	vs_c_thread.cpp
	vs_c_init.cpp
	vs_c_status.cpp
	vs_c_productionState.cpp
	vs_c_liveImage.cpp
	vs_c_format.cpp
	vs_c_marker.cpp
	vs_c_versionCheckVsPs.cpp
	connectionHandler.cpp
	${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp
)

add_executable(VsCommunication ${VS_COMMUNICATION_SERVER_SOURCES})

target_precompile_headers(VsCommunication 
    PRIVATE 
    ${CMAKE_SOURCE_DIR}/pch.h)


set_target_properties(VsCommunication
	PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

target_include_directories(VsCommunication 
	PRIVATE
	${CMAKE_SOURCE_DIR}/src/common
	${CMAKE_SOURCE_DIR}/libs
)

target_link_libraries(VsCommunication 
	PRIVATE 
	RedisClient 
	dbFacade
	stdc++fs 
	spdlog
	-pthread 
	proto_messages_lib
)




