find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED system)

add_executable(missionController 
    main.cpp
    missionController.cpp
    ${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp
)

set_target_properties(missionController
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

target_include_directories(missionController
    PRIVATE
    ${CMAKE_SOURCE_DIR}/libs
    ${CMAKE_SOURCE_DIR}/src/detectPickableObjects
    ${CMAKE_SOURCE_DIR}/src/common
    ${CMAKE_SOURCE_DIR}/src/common/logging
)

target_link_libraries(missionController
    PRIVATE
    spdlog
    ${OpenCV_LIBS}
    Boost::system
    RedisClient    
    dbFacade
)

