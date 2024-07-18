message("Build " ${CMAKE_CURRENT_SOURCE_DIR})

find_package(OpenCV REQUIRED)

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

set(TEST_SOURCES "udp_handler_test.h" "udp_handler_test.cpp")

include_directories(${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})

add_executable(TEST_UdpHandler ${TEST_SOURCES})

add_test(NAME TEST_UdpHandler COMMAND TEST_UdpHandler)
 
target_link_libraries(TEST_UdpHandler PUBLIC VsUdpHandler  gtest ${OpenCV_LIBS})
