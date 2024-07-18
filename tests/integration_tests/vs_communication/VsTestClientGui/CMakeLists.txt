find_package(OpenCV REQUIRED)

add_executable(VsStTestClientGui vs_c_testClient.cpp
								${CMAKE_SOURCE_DIR}/src/vs_communication/vs_c_tcpStream.cpp
								${CMAKE_SOURCE_DIR}/src/vs_communication/vs_c_tcpConnector.cpp
)

target_include_directories(VsStTestClientGui 
							PRIVATE 
							${OpenCV_INCLUDE_DIRS}
							${CMAKE_SOURCE_DIR}/src/vs_communication
							${CMAKE_BINARY_DIR}/src/vs_communication #for *.pb.h files
							${CMAKE_SOURCE_DIR}/src/common
							${CMAKE_SOURCE_DIR}/libs)

target_link_libraries(VsStTestClientGui 
	stdc++fs 
	-pthread 
	RedisClient 
	${OpenCV_LIBS}
	${Boost_LIBRARIES} 
	proto_messages_lib
	gtest
)


