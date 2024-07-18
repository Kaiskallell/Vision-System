find_package(OpenCV REQUIRED)
add_executable(TEST_Yolo 
                yolo_test.cpp
                ${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp)

set_target_properties(TEST_Yolo
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

target_include_directories(TEST_Yolo
                            PRIVATE
                            ${OpenCV_INCLUDE_DIRS}
                            PUBLIC 
                            ${CMAKE_SOURCE_DIR}/libs
                            ${CMAKE_SOURCE_DIR}/src/common
                            ${CMAKE_SOURCE_DIR}/src/common/logging
                            )
 
target_link_libraries(TEST_Yolo PRIVATE
                            gtest
                            ${OpenCV_LIBS}
                            stdc++fs
                            yolo
                            spdlog
                            )

add_test(NAME TEST_Yolo COMMAND TEST_Yolo)
