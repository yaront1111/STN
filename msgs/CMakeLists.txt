# Message definitions for AION

# Protobuf compilation
find_package(Protobuf REQUIRED)

# Generate C++ files from proto
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS aion_messages.proto)

# Create library with generated protobuf code
add_library(aion_msgs STATIC ${PROTO_SRCS} ${PROTO_HDRS})

target_include_directories(aion_msgs PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}  # For generated headers
  ${PROTOBUF_INCLUDE_DIRS}
)

target_link_libraries(aion_msgs PUBLIC
  ${PROTOBUF_LIBRARIES}
)

# Make generated headers available to other targets
set(AION_MSGS_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR} PARENT_SCOPE)