find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(ids_peak REQUIRED)
find_package(ids_peak_ipl REQUIRED)

add_library (vsGV5040FA
	${SAMPLE_TARGET_DEFINITION_ADDENDUM}
	vs_gv5040FA.cpp 
    ${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp
)

target_include_directories (vsGV5040FA
	PRIVATE 
	${${LIBRARY_NAME_UPPER_VISION_API}_INCLUDE_DIR}
	${${LIBRARY_NAME_LIBIMG_UPPER}_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_SOURCE_DIR}/libs
	PUBLIC
	${CMAKE_SOURCE_DIR}/src/common
    ${CMAKE_SOURCE_DIR}/src/common/logging
)

target_link_libraries (vsGV5040FA
	PRIVATE
	${CMAKE_THREAD_LIBS_INIT}
	${OpenCV_LIBS}
	RedisClient
    spdlog
	ids_peak
	ids_peak_ipl
)



