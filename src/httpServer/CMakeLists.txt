find_package(Boost REQUIRED system)
find_package(OpenCV REQUIRED)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

add_library(httpServer server.cpp )
target_include_directories(httpServer
	PUBLIC
	${Boost_INCLUDE_DIR}
	${CMAKE_SOURCE_DIR}/src/common
)

target_link_libraries(httpServer
	PUBLIC
	${Boost_LIBRARIES} 
	stdc++fs
	crypto
)

add_executable(vsHttpServer main.cpp)

set_target_properties(vsHttpServer
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

target_link_libraries(vsHttpServer
	PUBLIC
	httpServer
	${OpenCV_LIBS}
)