find_package(OpenCV)

add_library(cameraFactory cameraFactory.cpp
                          ${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp)
target_include_directories(cameraFactory 
                            PRIVATE
                            ${CMAKE_SOURCE_DIR}/libs
                            ${CMAKE_SOURCE_DIR}/src/common
                            ${CMAKE_SOURCE_DIR}/src/common/logging
                            ${CMAKE_SOURCE_DIR}/src/common/gv5040FA
                            ${CMAKE_SOURCE_DIR}/src/common/mockCamera
                            ${CMAKE_SOURCE_DIR}/src/common/baumerCam
                            ${CMAKE_SOURCE_DIR}/src/common/CREStereoDepthCamDecorator
                            PUBLIC
                            ${OpenCV_INLCUDE_DIRS})

target_link_libraries(cameraFactory 
                        PRIVATE
                        mockCamera
                        vsGV5040FA
                        baumerCam
                        creStereoDepthCamDecorator
                        ${OpenCV_LIBS}
                        spdlog
                        )            
