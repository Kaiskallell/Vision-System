set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED TRUE)

find_package(CUDAToolkit REQUIRED)
find_package(OpenCV REQUIRED)

add_library(tensorRtHelpers 
    SHARED 
    trtInference.cpp
    trtLogging.cpp            
    entropyCalibrator.cpp
   	${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp
)

target_include_directories(tensorRtHelpers 
    PRIVATE
    ${OpenCV_INCLUDE_DIRS}    
    ${CMAKE_SOURCE_DIR}/src/common/logging
    PUBLIC 
    ${CUDAToolkit_INCLUDE_DIRS}                                  
    ${CMAKE_SOURCE_DIR}/libs
)

target_link_libraries(tensorRtHelpers
    PRIVATE
    nvinfer
    spdlog
    nvonnxparser
    CUDA::cudart
    stdc++fs
    ${CUDA_LIBRARIES}
    ${OpenCV_LIBS})
