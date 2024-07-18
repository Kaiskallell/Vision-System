find_package(OpenCV REQUIRED)
find_package (Eigen3)

add_library(trackingInterface trackingInterface.h)
set_target_properties(trackingInterface PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(trackingInterface PRIVATE
                            ${CMAKE_CURRENT_SOURCE_DIR}
                            ${CMAKE_SOURCE_DIR}/src/common
                            ${CMAKE_SOURCE_DIR}/src/vs_communication)
target_link_libraries(trackingInterface
            ${OpenCV_LIBS}
)

add_library(detectPlace placeConveyorDetection.cpp
                        ${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp)

target_include_directories(detectPlace PRIVATE
                            ${CMAKE_CURRENT_SOURCE_DIR}
                            ${CMAKE_SOURCE_DIR}/libs
                            ${CMAKE_SOURCE_DIR}/src/common
                            ${CMAKE_SOURCE_DIR}/src/common/logging
                            ${CMAKE_SOURCE_DIR}/src/vs_communication)

target_link_libraries(detectPlace
    cameraFactory
    misc
    ${OpenCV_LIBS}
    VsUdpHandler
    poseEstimation
    yoloDLCPipeline
    dbFacade
    spdlog
    trackingInterface
)

add_library(tracking   
                    objTracking.cpp
                    ${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp
)

target_include_directories(tracking
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/libs
    ${CMAKE_SOURCE_DIR}/src/common
    ${CMAKE_SOURCE_DIR}/src/common/logging
    ${CMAKE_SOURCE_DIR}/src/vs_communication
    PUBLIC
    ${CMAKE_SOURCE_DIR}/libs/external/hungarian-algorithm
)

target_link_libraries(tracking
    #PRIVATE
    PUBLIC
    misc
    Eigen3::Eigen
    hungarianAlgo
    #PUBLIC
    spdlog
    ${OpenCV_LIBS}
    dbFacade
    trackingInterface
    misc
)

add_executable(placeConveyorTracking 
    main_placeConveyorTracking.cpp
    ${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp
)

set_target_properties(placeConveyorTracking
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

target_include_directories(placeConveyorTracking
    PRIVATE
    ${CMAKE_SOURCE_DIR}/src/common
    ${CMAKE_SOURCE_DIR}/libs
    ${CMAKE_SOURCE_DIR}/src/common/logging
    ${CMAKE_SOURCE_DIR}/src/vs_communication

)

target_link_libraries(placeConveyorTracking
    PRIVATE
    tracking
    detectPlace
    stdc++fs
)

