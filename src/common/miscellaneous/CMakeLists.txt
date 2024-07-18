find_package(OpenCV)

add_library(misc miscellaneous.cpp
                 )
target_include_directories(misc 
                            PRIVATE
                            ${CMAKE_SOURCE_DIR}/src/common/logging
                            ${CMAKE_SOURCE_DIR}/src/common
                            ${CMAKE_SOURCE_DIR}/libs
                            PUBLIC
                            ${OpenCV_INLCUDE_DIRS})

target_link_libraries(misc 
                        PRIVATE
                        ${OpenCV_LIBS}
                        spdlog
                        )            
