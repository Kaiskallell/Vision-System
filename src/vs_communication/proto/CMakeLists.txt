find_package(Protobuf REQUIRED)

#build protocol buffers
set(PROTO_FILES
	#ST
	STVS/GetStatus.proto
	STVS/Init.proto
	STVS/Nack.proto
	STVS/StartProduction.proto

	#PS
	VSPS/Marker.proto
	VSPS/LiveImage.proto
	VSPS/Format.proto
	VSPS/VersionCheckVsPs.proto
)

#automatically generate c++ classes from the definitions in the proto files
PROTOBUF_GENERATE_CPP(GENERATED_PROTO_SRC GENERATED_PROTO_HEADER ${PROTO_FILES})

add_library(proto_messages_lib 	
	${GENERATED_PROTO_SRC}
	${GENERATED_PROTO_HEADER})

target_include_directories(proto_messages_lib 
							PUBLIC
							${PROTOBUF_INCLUDE_DIRS}
							${CMAKE_CURRENT_BINARY_DIR}	)

target_link_libraries(proto_messages_lib PRIVATE
					  	${PROTOBUF_LIBRARIES}
					  )



