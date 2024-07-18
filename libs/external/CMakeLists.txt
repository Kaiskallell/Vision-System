# add external libraries
add_subdirectory(googletest)
set(SPDLOG_BUILD_SHARED ON)
add_subdirectory(spdlog)
add_subdirectory(hungarian-algorithm)
# TODO: replace redis_client with this external lib
#add_subdirectory(redis-plus-plus)
