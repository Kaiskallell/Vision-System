find_package(OpenCV REQUIRED)
find_package(CUDAToolkit  REQUIRED)

add_library(inferDLC 
	inferDLC.cpp
	${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp
)

target_include_directories(inferDLC
	PRIVATE
	${OpenCV_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/libs
	${CMAKE_SOURCE_DIR}/src/common
	${CMAKE_SOURCE_DIR}/src/common/logging
    ${CMAKE_SOURCE_DIR}/src/common/miscellaneous
	PUBLIC
	${CMAKE_SOURCE_DIR}/src/common/tensorRtHelpers
	${CMAKE_CURRENT_SOURCE_DIR}
	${CUDAToolkit_INCLUDE_DIRS}
)

target_link_libraries(inferDLC
    PRIVATE
	nvinfer
	nvonnxparser
	CUDA::cudart
	${CUDA_LIBRARIES}
	tensorRtHelpers
	misc
	spdlog
	PUBLIC
	stdc++fs
	${OpenCV_LIBS}
)
