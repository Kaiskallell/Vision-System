find_package(OpenCV REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(NEOAPI_ROOT_DIR ${CMAKE_SOURCE_DIR}/libs/external/Baumer_neoAPI_1.2.0_lin_aarch64_cpp)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(NEOAPI_ROOT_DIR ${CMAKE_SOURCE_DIR}/libs/external/Baumer_neoAPI_1.2.0_lin_x86_64_cpp)
else()
    message(FATAL_ERROR "Unsupported architecture")
endif()

find_library(LIB_NEOAPI_CPP NAMES neoapi_cpp libneoapi_cpp.so libneoapi_cpp.so.1.2 HINTS ${NEOAPI_ROOT_DIR}/lib/)

add_library(baumerCam SHARED baumerCam.cpp
                     ${CMAKE_SOURCE_DIR}/src/common/logging/logging.cpp
)

target_include_directories(baumerCam 
                            PRIVATE
                            SYSTEM 
                            ${OpenCV_INCLUDE_DIRS}
                            ${CMAKE_SOURCE_DIR}/src/common/logging
                            ${NEOAPI_ROOT_DIR}/include
                            ${CMAKE_SOURCE_DIR}/src/common
                            ${CMAKE_SOURCE_DIR}/libs)

                
target_link_directories(baumerCam 
                            PRIVATE 
                            ${NEOAPI_ROOT_DIR}/lib )

target_link_libraries(baumerCam 
                            PRIVATE 
                            ${LIB_NEOAPI_CPP}
                            spdlog
                            ${OpenCV_LIBS}
                            )