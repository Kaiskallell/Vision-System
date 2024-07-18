find_package(CUDAToolkit REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)


add_definitions(${PCL_DEFINITIONS})

add_library(poseEstimation 
	poseEstimation.cpp
	${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp
)

target_include_directories(poseEstimation
	PRIVATE
	${PCL_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/src/common
	${CMAKE_SOURCE_DIR}/src/common/logging
	${CMAKE_SOURCE_DIR}/src/common/mathHelperFuncs
	${CMAKE_SOURCE_DIR}/libs
	
	PUBLIC
	${CMAKE_SOURCE_DIR}/src/common/config
	${OpenCV_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(poseEstimation 
	PRIVATE
	${PCL_LIBRARIES} 
	CUDA::cudart 
	mathHelperFuncs 
	RedisClient
	Eigen3::Eigen
	spdlog
	PUBLIC
	${OpenCV_LIBS} 
)
