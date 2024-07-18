find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(PCL 1.8 REQUIRED QUIET)

add_definitions(${PCL_DEFINITIONS})

add_executable(TEST_PoseEstimation 
                pose_estimation_test.cpp
                ${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp)

set_target_properties(TEST_PoseEstimation
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

target_include_directories(TEST_PoseEstimation
                            PRIVATE
                            ${PCL_INCLUDE_DIRS}
                            ${OpenCV_INCLUDE_DIRS}
                            ${CMAKE_SOURCE_DIR}/src/common
                            ${CMAKE_SOURCE_DIR}/libs
                            ${CMAKE_SOURCE_DIR}/src/common/logging
                            )
 
target_link_libraries(TEST_PoseEstimation PUBLIC
                            poseEstimation
                            gtest
                            ${OpenCV_LIBS}
                            RedisClient
                            spdlog
                            Eigen3::Eigen
                            CUDA::cudart 
                            ${PCL_LIBRARIES} )

add_test(NAME TEST_PoseEstimation COMMAND TEST_PoseEstimation)
