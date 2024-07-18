find_package(OpenCV REQUIRED)
add_executable(TEST_HandEyeAlgo3dBoard 
                handEyeAlgo3dBoard_test.cpp
                ${CMAKE_SOURCE_DIR}/src/handEyeCalibration/handEyeAlgo.cpp
                ${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp)

set_target_properties(TEST_HandEyeAlgo3dBoard
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

target_include_directories(TEST_HandEyeAlgo3dBoard
                            PRIVATE
                            ${OpenCV_INCLUDE_DIRS}
                            ${CMAKE_SOURCE_DIR}/libs
                            ${CMAKE_SOURCE_DIR}/src/common
                            ${CMAKE_SOURCE_DIR}/src/common/miscellaneous
                            ${CMAKE_SOURCE_DIR}/src/common/logging
                            ${CMAKE_SOURCE_DIR}/src/handEyeCalibration
                            )
 
target_link_libraries(TEST_HandEyeAlgo3dBoard 
                            PUBLIC
                            gtest
                            spdlog
                            ${OpenCV_LIBS}
                            stdc++fs
                            misc
                    )

add_test(NAME TEST_HandEyeAlgo3dBoard COMMAND TEST_HandEyeAlgo3dBoard)