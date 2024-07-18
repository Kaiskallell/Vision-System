
find_package(OpenCV)

add_library(yolo      yolo.cpp 
                        ${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp
                        )

target_include_directories(yolo PRIVATE 
                                    ${OpenCV_INCLUDE_DIRS}                            
                                    ${CMAKE_SOURCE_DIR}/libs
                                    ${CMAKE_SOURCE_DIR}/src/common/tensorRtHelpers
                                   PUBLIC 
                                    ${CMAKE_SOURCE_DIR}/src/common/logging
                                    ${CMAKE_SOURCE_DIR}/src/common
                                    ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(yolo PRIVATE
                            tensorRtHelpers
                            ${OpenCV_LIBS}
                            stdc++fs
                            PUBLIC
                            spdlog
                            )


