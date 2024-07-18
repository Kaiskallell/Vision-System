add_library(poseEstimationIPA 
	poseEstimationIPA.cpp
	${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp
)

target_include_directories(poseEstimationIPA
    PRIVATE
    ${CMAKE_SOURCE_DIR}/src/common/logging
    ${CMAKE_SOURCE_DIR}/src/common
    ${CMAKE_SOURCE_DIR}/src/common/mathHelperFuncs
    ${CMAKE_SOURCE_DIR}/src/common/poseEstimation
    ${CMAKE_SOURCE_DIR}/libs
)

target_link_libraries(poseEstimationIPA 
	PRIVATE
    poseEstimation
    mathHelperFuncs
    spdlog
    ${CMAKE_SOURCE_DIR}/libs/external/libGrippingPointDetection.so
)
