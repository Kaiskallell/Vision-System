find_package(OpenCV REQUIRED)

add_library(yoloDLCPipeline 
	yoloDLCPipeline.cpp
)

target_include_directories(yoloDLCPipeline
	PRIVATE
	${OpenCV_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/libs
	${CMAKE_SOURCE_DIR}/src/common/logging
	${CMAKE_SOURCE_DIR}/src/common/deeplabcut
	${CMAKE_SOURCE_DIR}/src/common/yolo
    ${CMAKE_SOURCE_DIR}/src/common/miscellaneous
	PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(yoloDLCPipeline
    PRIVATE 
	yolo
	inferDLC
	misc
	PUBLIC
	stdc++fs
	${OpenCV_LIBS}
)
