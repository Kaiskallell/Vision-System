message("Build " ${CMAKE_CURRENT_SOURCE_DIR})

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(BINARY TEST_GetStatus)

file(GLOB COMMUNICATION_TEST getStatus_test.cpp)
set(TEST_SOURCES ${COMMUNICATION_TEST})

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(${BINARY} ${TEST_SOURCES})

add_test(NAME ${BINARY} COMMAND ${BINARY})
 
target_link_libraries(${BINARY} PUBLIC test_communication_lib gtest)