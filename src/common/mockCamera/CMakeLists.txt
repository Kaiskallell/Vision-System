find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(mockCamera 
	mockCamera.cpp
	${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp
)

target_include_directories(mockCamera
	PRIVATE
	${OpenCV_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/src/common
	${CMAKE_SOURCE_DIR}/src/common/logging
	${CMAKE_SOURCE_DIR}/libs
	PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(mockCamera
	PRIVATE
	stdc++fs
	${OpenCV_LIBS}
	RedisClient
	spdlog
	
)
