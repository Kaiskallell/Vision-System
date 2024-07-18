find_package(Boost REQUIRED system)

add_executable(TEST_httpServer  httpServerTest.cpp
								httpClient.hpp
								)

set_target_properties(TEST_httpServer
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

target_include_directories(TEST_httpServer 
							PRIVATE 
							${CMAKE_SOURCE_DIR}/libs
							${CMAKE_SOURCE_DIR}/src/common
                            ${CMAKE_SOURCE_DIR}/src/httpServer
							${CMAKE_SOURCE_DIR}/tests/integration_tests/
							)

target_link_libraries(TEST_httpServer 
PRIVATE
    httpServer
	stdc++fs
	Boost::system
	gtest
)

add_test(NAME TEST_httpServer COMMAND TEST_httpServer)

