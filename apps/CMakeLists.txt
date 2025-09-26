add_executable(aion_node aion_node.cpp)

target_link_libraries(aion_node PRIVATE
  aion_core
  measurements
  terrain
  yaml-cpp
  Threads::Threads
)

# Install executable
install(TARGETS aion_node
  RUNTIME DESTINATION bin
)