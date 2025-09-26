# Unit tests for AION core components

# Time utilities test
add_executable(test_time_utils test_time_utils.cpp)
target_link_libraries(test_time_utils
  aion_core
  GTest::gtest
  GTest::gtest_main
  Threads::Threads
)
add_test(NAME TimeUtilsTest COMMAND test_time_utils)

# IMU preintegration test
add_executable(test_imu_preintegration test_imu_preintegration.cpp)
target_link_libraries(test_imu_preintegration
  aion_core
  GTest::gtest
  GTest::gtest_main
  Threads::Threads
)
add_test(NAME IMUPreintegrationTest COMMAND test_imu_preintegration)