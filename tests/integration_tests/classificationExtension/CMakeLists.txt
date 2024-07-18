find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED system)

add_executable(TEST_classificationExtension classificationExtension_test.h
classificationExtension_test.cpp)

set_target_properties(TEST_classificationExtension
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

target_include_directories(TEST_classificationExtension 
							PRIVATE 
							${OpenCV_INCLUDE_DIRS}
							${Boost_INCLUDE_DIR}
							${CMAKE_SOURCE_DIR}/libs
							${CMAKE_SOURCE_DIR}/tests/integration_tests/tc3Mock
							)

target_link_libraries(TEST_classificationExtension 
	${OpenCV_LIBS}
	${Boost_LIBRARIES} 
	gtest
	tc3Mock
	dbFacade
	spdlog
)

add_test(NAME TEST_classificationExtension COMMAND TEST_classificationExtension)

