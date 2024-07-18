find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED)

add_library(mathHelperFuncs 
	mathHelperFuncs.cpp 
	mathHelperFuncs.h
)

add_definitions(${PCL_DEFINITIONS})

target_include_directories(mathHelperFuncs
	PRIVATE
	${OpenCV_INCLUDE_DIRS}
	PUBLIC
	${PCL_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/src/common
)

target_link_libraries(mathHelperFuncs 
	PRIVATE
	${OpenCV_LIBS}
	${PCL_LIBRARIES} 
)
