find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED system)

set(VS_COMMUNICATION_CLIENT_SOURCES 
	vs_c_testClient.h
	vs_c_testClient.cpp
	${CMAKE_SOURCE_DIR}/src/vs_communication/vs_c_tcpStream.cpp
	${CMAKE_SOURCE_DIR}/src/vs_communication/vs_c_tcpConnector.cpp
)

add_executable(TEST_VsStCommunication ${VS_COMMUNICATION_CLIENT_SOURCES})

set_target_properties(TEST_VsStCommunication
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

target_include_directories(TEST_VsStCommunication 
							PRIVATE 
							${OpenCV_INCLUDE_DIRS}
							${Boost_INCLUDE_DIR}
							${CMAKE_SOURCE_DIR}/src/vs_communication
							${CMAKE_BINARY_DIR}/src/vs_communication #for *.pb.h files
							${CMAKE_SOURCE_DIR}/src/common
							${CMAKE_SOURCE_DIR}/libs
							)

target_link_libraries(TEST_VsStCommunication 
	stdc++fs 
	-pthread 
	RedisClient 
	${OpenCV_LIBS}
	${Boost_LIBRARIES} 
	proto_messages_lib
	gtest
)

add_test(NAME TEST_VsStCommunication COMMAND TEST_VsStCommunication)

