find_package(OpenCV REQUIRED)

add_library(stereoDepth stereoDepth.cpp 
                        ${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp
                        )

target_include_directories(stereoDepth PRIVATE 
                                    ${OpenCV_INCLUDE_DIRS}
                                   PUBLIC 
                                    ${CMAKE_SOURCE_DIR}/src/common/logging
                                    ${CMAKE_SOURCE_DIR}/src/common
                                    ${CMAKE_SOURCE_DIR}/libs
                                    ${CMAKE_SOURCE_DIR}/src/common/tensorRtHelpers
                                    ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(stereoDepth PRIVATE
                            tensorRtHelpers
                            ${OpenCV_LIBS}
                            stdc++fs
                            PUBLIC
                            spdlog
                            )
