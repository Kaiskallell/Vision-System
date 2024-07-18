find_package(OpenCV REQUIRED)
add_executable(TEST_Yolo_FILTERBYSIZE 
                yolo_filterBySize_test.cpp
                ${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp)

set_target_properties(TEST_Yolo_FILTERBYSIZE
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

target_include_directories(TEST_Yolo_FILTERBYSIZE
                            PRIVATE
                            ${OpenCV_INCLUDE_DIRS}
                            PUBLIC 
                            ${CMAKE_SOURCE_DIR}/libs
                            ${CMAKE_SOURCE_DIR}/src/common
                            ${CMAKE_SOURCE_DIR}/src/common/logging
                            )
 
target_link_libraries(TEST_Yolo_FILTERBYSIZE PRIVATE
                            gtest
                            ${OpenCV_LIBS}
                            stdc++fs
                            yolo
                            spdlog
                            )

add_test(NAME TEST_Yolo_FILTERBYSIZE COMMAND TEST_Yolo_FILTERBYSIZE)
