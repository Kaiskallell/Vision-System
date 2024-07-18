find_package(OpenCV REQUIRED)

add_library(creStereoDepthCamDecorator creStereoDepthCamDecorator.cpp
                                        ${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp
                                        )

target_include_directories(creStereoDepthCamDecorator PRIVATE 
                                   ${OpenCV_INCLUDE_DIRS}  
                                   PUBLIC 
                                   ${CMAKE_SOURCE_DIR}/src/common/miscellaneous
                                   ${CMAKE_SOURCE_DIR}/src/common/logging
                                   ${CMAKE_SOURCE_DIR}/src/common
                                   ${CMAKE_SOURCE_DIR}/libs
)

target_link_libraries(creStereoDepthCamDecorator PRIVATE 
                                                stereoDepth
                                                ${OpenCV_LIBS}
                                                stdc++fs
                                                misc
                                                PUBLIC
                                                spdlog)