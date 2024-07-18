find_package(OpenCV REQUIRED)
add_executable(TEST_stereoDepthEstimator 
                stereoDepthEstimator_test.cpp
                ${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp)

set_target_properties(TEST_stereoDepthEstimator
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

target_include_directories(TEST_stereoDepthEstimator
                            PRIVATE
                            ${OpenCV_INCLUDE_DIRS}
                            ${CMAKE_SOURCE_DIR}/src/common
                            ${CMAKE_SOURCE_DIR}/src/common/logging
                            ${CMAKE_SOURCE_DIR}/src/common/stereoDepth
                            )
 
target_link_libraries(TEST_stereoDepthEstimator PUBLIC
                            gtest
                            ${OpenCV_LIBS}
                            spdlog
                            stereoDepth
                            )

add_test(NAME TEST_stereoDepthEstimator COMMAND TEST_stereoDepthEstimator)
