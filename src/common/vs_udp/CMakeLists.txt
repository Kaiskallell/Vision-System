find_package(Boost REQUIRED system)
find_package(OpenCV REQUIRED) # this is only required for timing. TODO: remove

add_library(VsUdpHandler 
	udphandler.cpp
	udpReceiver.cpp
	mockUdpHandler.hpp
	${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp
)

target_include_directories(VsUdpHandler
	PRIVATE
	${OpenCV_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/src/common/logging
	${CMAKE_SOURCE_DIR}/src/common
	${CMAKE_SOURCE_DIR}/libs
	PUBLIC
	${Boost_INCLUDE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(VsUdpHandler
	PRIVATE
	pthread 
	spdlog
	${OpenCV_LIBS}
	dbFacade
	PUBLIC
	${Boost_LIBRARIES} 
)

