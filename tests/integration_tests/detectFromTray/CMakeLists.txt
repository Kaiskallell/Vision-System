find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED QUIET)

add_executable(testDetectFromTray
                detectFromTrayTest.cpp
				${CMAKE_SOURCE_DIR}/src/detectPickableObjects/detectFromTray.cpp
				${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp)



target_precompile_headers(testDetectFromTray 
	PRIVATE 
	${CMAKE_SOURCE_DIR}/pch.h
)

set_target_properties(testDetectFromTray
	PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

target_include_directories(testDetectFromTray
	PRIVATE
	${OpenCV_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/src/detectPickableObjects
	${PCL_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/libs
    ${CMAKE_SOURCE_DIR}/src/common
    ${CMAKE_SOURCE_DIR}/src/common/logging
    ${CMAKE_SOURCE_DIR}/src/common/yoloDLCPipeline
	${CMAKE_SOURCE_DIR}/src/common/depthGan
	${CMAKE_SOURCE_DIR}/src/common/miscellaneous
)

target_link_libraries(testDetectFromTray
	PRIVATE
    spdlog
    ${OpenCV_LIBS}
    cameraFactory
    RedisClient
    VsUdpHandler
    poseEstimation
    yoloDLCPipeline
    dbFacade
	gtest
	depthGan
	proto_messages_lib
	misc
	spdlog
) 
