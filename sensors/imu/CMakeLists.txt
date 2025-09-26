# IMU driver library

add_library(aion_imu STATIC
  imu_driver.cpp
  imu_simulator.cpp
)

target_link_libraries(aion_imu PUBLIC
  aion_core
  Eigen3::Eigen
  ${YAML_CPP_LIBRARIES}
)

target_include_directories(aion_imu PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../..>
  $<INSTALL_INTERFACE:include>
)