find_package(OpenCV REQUIRED)

add_library(detecting 
    detecting.cpp
    pickconveyor.cpp
    detectFromTray.cpp
    ipaDetection.cpp
    stechtest.cpp
    ${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp
)

target_precompile_headers(detecting 
    PRIVATE 
    ${CMAKE_SOURCE_DIR}/pch.h)

target_include_directories(detecting
    PRIVATE
    ${CMAKE_SOURCE_DIR}/libs
    ${CMAKE_SOURCE_DIR}/src/common
    ${CMAKE_SOURCE_DIR}/src/common/logging
    ${CMAKE_SOURCE_DIR}/src/common/yoloDLCPipeline
    ${CMAKE_SOURCE_DIR}/src/common/miscellaneous
    ${CMAKE_SOURCE_DIR}/src/common/poseEstimationIPA
    ${CMAKE_SOURCE_DIR}/src/common/depthOnlyOnDetections
    ${CMAKE_SOURCE_DIR}/src/vs_communication
)

target_link_libraries(detecting
    PRIVATE
    spdlog
    ${OpenCV_LIBS}
    cameraFactory
    RedisClient
    VsUdpHandler
    poseEstimation
    yoloDLCPipeline
    dbFacade
    misc
    depthOnlyOnDetections
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_link_libraries(detecting
    PRIVATE
        poseEstimationIPA
    )
endif()

add_executable(detectPickableObjects 
    main_picking.cpp
    ${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp
)

set_target_properties(detectPickableObjects
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

target_include_directories(detectPickableObjects
    PRIVATE
    ${CMAKE_SOURCE_DIR}/libs
    ${CMAKE_SOURCE_DIR}/src/common/logging
    ${CMAKE_SOURCE_DIR}/src/common
)

target_link_libraries(detectPickableObjects PRIVATE 
    detecting    
    spdlog 
    ${OpenCV_LIBS}
)

