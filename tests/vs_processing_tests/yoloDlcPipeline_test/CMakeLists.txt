find_package(OpenCV REQUIRED)

add_executable(TEST_YoloDlcPipeline
                yoloDlcPipeline_test.cpp
                ${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp)

set_target_properties(TEST_YoloDlcPipeline
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

target_include_directories(TEST_YoloDlcPipeline
                            PRIVATE
                            ${OpenCV_INCLUDE_DIRS}
                            ${CMAKE_SOURCE_DIR}/src/common
                            ${CMAKE_SOURCE_DIR}/src/common/yoloDLCPipeline
                            ${CMAKE_SOURCE_DIR}/libs
                            ${CMAKE_SOURCE_DIR}/src/common/logging
                            )
 
target_link_libraries(TEST_YoloDlcPipeline PUBLIC
                            gtest
                            yoloDLCPipeline
                            ${OpenCV_LIBS}
                            spdlog
                            )

add_test(NAME TEST_YoloDlcPipeline COMMAND TEST_YoloDlcPipeline)
