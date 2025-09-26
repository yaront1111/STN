# AION Replay Tool

add_executable(aion_replay aion_replay.cpp)

target_link_libraries(aion_replay
  aion_core
  aion_msgs
  ${PROTOBUF_LIBRARIES}
  spdlog::spdlog
)

install(TARGETS aion_replay
  RUNTIME DESTINATION bin
)