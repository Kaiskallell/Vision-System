cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(VisionSystem LANGUAGES CXX CUDA)
option(BUILD_TESTS "Build unit and integration tests" OFF) # cmake -D BUILD_TESTS=ON ..

set(CMAKE_BUILD_TYPE Debug) # For an unoptimized build with assertions and debug info.
#set(CMAKE_BUILD_TYPE RelWithDebInfo) # For an optimized build with no assertions but with debug info.
#set(CMAKE_BUILD_TYPE MinSizeRel) # For a build optimized for size instead of speed.
#set(CMAKE_BUILD_TYPE Release) # For an optimized build with no assertions or debug info.

# Check if the build type is set to Release
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    # Enable optimization flags
    add_compile_options(-O3)

    # Enable architecture-specific optimizations
    add_compile_options(-march=native)

    # Enable link-time optimization
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -flto")

    # Disable frame pointer
    add_compile_options(-fomit-frame-pointer)

    # Disable runtime type information
    #add_compile_options(-fno-rtti)

    # Disable stack protection
    add_compile_options(-fno-stack-protector)
endif()

#include(scripts/CheckGit.cmake)
#CheckGitSetup()

# add external libraries
add_subdirectory(libs/external)

# Targets intended for the robot
add_subdirectory(src/common)
add_subdirectory(src/vs_communication)
add_subdirectory(src/detectPickableObjects)
add_subdirectory(src/missionControl)
add_subdirectory(src/handEyeCalibration)
add_subdirectory(src/httpServer)
add_subdirectory(src/placeConveyorTracking)
add_subdirectory(src/classificatorExtension)

# Tests
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif(BUILD_TESTS)
