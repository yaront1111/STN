# Sensor drivers for AION

add_library(aion_sensors INTERFACE)

# Add sensor drivers as they become available
add_subdirectory(imu)
# add_subdirectory(radar)
# add_subdirectory(ralt)
# add_subdirectory(camera)
# add_subdirectory(lidar)
# add_subdirectory(baro_mag)

target_link_libraries(aion_sensors INTERFACE
  aion_core
)