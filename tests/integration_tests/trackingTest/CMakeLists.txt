find_package(OpenCV REQUIRED)

add_executable(TEST_tracking
                trackingTest.cpp
				${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp
				)


set_target_properties(TEST_tracking
	PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

target_include_directories(TEST_tracking
	PRIVATE
	${OpenCV_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/libs
	${CMAKE_SOURCE_DIR}/src/common/
    ${CMAKE_SOURCE_DIR}/src/common/logging
	${CMAKE_SOURCE_DIR}/src/vs_communication
	${CMAKE_SOURCE_DIR}/src/placeConveyorTracking
)

target_link_libraries(TEST_tracking
	PRIVATE
	spdlog
	gtest
    ${OpenCV_LIBS}
	tracking
	dbFacade
	proto_messages_lib
	misc
	) 

add_test(NAME TEST_tracking COMMAND TEST_tracking)
