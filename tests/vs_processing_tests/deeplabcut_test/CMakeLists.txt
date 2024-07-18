find_package(OpenCV REQUIRED)
add_executable(TEST_dlc
                deeplabcut_test.cpp
                ${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp)

set_target_properties(TEST_dlc
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

target_include_directories(TEST_dlc
                            PRIVATE
                            ${OpenCV_INCLUDE_DIRS}
                            PUBLIC 
                            ${CMAKE_SOURCE_DIR}/libs
                            ${CMAKE_SOURCE_DIR}/src/common
                            ${CMAKE_SOURCE_DIR}/src/common/tensorRtHelpers
                            ${CMAKE_SOURCE_DIR}/src/common/logging
                            )
 
target_link_libraries(TEST_dlc PRIVATE
                            gtest
                            ${OpenCV_LIBS}
                            stdc++fs
                            inferDLC
                            spdlog
                            )

add_test(NAME TEST_dlc COMMAND TEST_dlc)
