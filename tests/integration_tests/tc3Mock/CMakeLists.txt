
find_package(Boost REQUIRED system)

add_library(tc3Mock tc3Mock.cpp)
target_include_directories(tc3Mock 
                            PUBLIC
							${CMAKE_SOURCE_DIR}/src/common
                            ${CMAKE_SOURCE_DIR}/src/common/vs_udp )

target_link_libraries(tc3Mock
	PRIVATE
	${OpenCV_LIBS}
	${Boost_LIBRARIES} 
)


