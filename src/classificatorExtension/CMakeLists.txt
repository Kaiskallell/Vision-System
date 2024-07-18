find_package(OpenCV REQUIRED)


add_executable(classificatorExtension 
    main_classificatorExtension.cpp
    classificatorExtension.cpp
    ${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp
)

set_target_properties(classificatorExtension
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

target_include_directories(classificatorExtension
    PRIVATE
    ${CMAKE_SOURCE_DIR}/libs
    ${CMAKE_SOURCE_DIR}/src/common/logging
    ${CMAKE_SOURCE_DIR}/src/common
    ${CMAKE_SOURCE_DIR}/src/common/miscellaneous
    ${CMAKE_SOURCE_DIR}/src/vs_communication
)

target_link_libraries(classificatorExtension PRIVATE 
    spdlog 
    dbFacade
    yolo
    cameraFactory
    VsUdpHandler
    misc
    ${OpenCV_LIBS}
)