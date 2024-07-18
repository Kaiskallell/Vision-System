add_library(RedisClient 
	SHARED
	command.cpp
	command_args.h
	command_options.cpp
	connection.cpp
	connection_pool.cpp
	crc16.cpp
	errors.cpp
	pipeline.cpp
	redis.cpp
	redis_cluster.cpp
	reply.cpp
	sentinel.cpp
	shards.cpp
	shards_pool.cpp
	subscriber.cpp
	transaction.cpp
)

target_include_directories(RedisClient
	PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(RedisClient PUBLIC -lhiredis stdc++fs)

