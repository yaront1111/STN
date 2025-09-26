# Frontend modules for AION

add_library(aion_frontends INTERFACE)

# Add subdirectories as they become available
# add_subdirectory(trn)
# add_subdirectory(radar_odom)
# add_subdirectory(vio_event)
# add_subdirectory(lidar_odom)
# add_subdirectory(sop)
# add_subdirectory(uwb)

target_link_libraries(aion_frontends INTERFACE
  aion_core
)