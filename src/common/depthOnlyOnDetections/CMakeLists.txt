find_package(OpenCV REQUIRED)

add_library(depthOnlyOnDetections depthOnlyOnDetections.cpp
								${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp)

target_include_directories(depthOnlyOnDetections 
    PRIVATE
	${OpenCV_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/libs
	${CMAKE_SOURCE_DIR}/src/common/tensorRtHelpers
	PUBLIC
	${CMAKE_SOURCE_DIR}/src/common
   	${CMAKE_SOURCE_DIR}/src/common/logging
)

target_link_libraries(depthOnlyOnDetections
    PRIVATE
	tensorRtHelpers
	${OpenCV_LIBS}
    misc
    spdlog
)