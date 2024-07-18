find_package(OpenCV REQUIRED)

add_library(vmsMock vmsMock.cpp
					${CMAKE_SOURCE_DIR}/src/vs_communication/vs_c_tcpStream.cpp
					${CMAKE_SOURCE_DIR}/src/vs_communication/vs_c_tcpConnector.cpp
					${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp
					)

target_include_directories(vmsMock 
							PRIVATE 
							${CMAKE_SOURCE_DIR}/src/vs_communication
							${CMAKE_BINARY_DIR}/src/vs_communication # for *.pb.h files
							${CMAKE_SOURCE_DIR}/src/common
							${CMAKE_SOURCE_DIR}/libs
							PUBLIC
							${CMAKE_SOURCE_DIR}/src/vs_communication
							)

target_link_libraries(vmsMock PRIVATE
	stdc++fs 
	-pthread 
	proto_messages_lib
	PUBLIC
	spdlog
)
