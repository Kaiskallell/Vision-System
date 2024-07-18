find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED system)

add_executable(calibration 
    main_calibration.cpp
    calibration.cpp
    handEyeAlgo.cpp
    stereoCalibration.cpp
    ${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp
)

target_precompile_headers(calibration 
    PRIVATE 
    ${CMAKE_SOURCE_DIR}/pch.h)

set_target_properties(calibration
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

target_include_directories(calibration
    PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/libs
    ${CMAKE_SOURCE_DIR}/src/common
    ${CMAKE_SOURCE_DIR}/src/common/miscellaneous
    ${CMAKE_SOURCE_DIR}/src/common/cameraFactory
    ${CMAKE_SOURCE_DIR}/src/common/logging
)

target_link_libraries(calibration
    PRIVATE
    spdlog
    ${OpenCV_LIBS}
    Boost::system
    dbFacade
    cameraFactory
    misc
)

find_package(PCL 1.8 REQUIRED)

add_executable(stechtest 
    main_stechtest.cpp
    ${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp
)

target_include_directories(stechtest
    PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/libs
    ${CMAKE_SOURCE_DIR}/src/common
    ${CMAKE_SOURCE_DIR}/src/common/cameraFactory
    ${CMAKE_SOURCE_DIR}/src/common/logging
    ${CMAKE_SOURCE_DIR}/src/common/miscellaneous
    ${CMAKE_SOURCE_DIR}/src/common/mathHelperFuncs
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(stechtest
    PRIVATE
    spdlog
    ${OpenCV_LIBS}
    Boost::system
    cameraFactory
    mathHelperFuncs
    misc
)
