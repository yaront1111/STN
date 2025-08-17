cmake_minimum_required(VERSION 3.16)
project(stn_v0_1 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)

add_executable(stn_demo
  core/main.cpp
  cpp/core/types.h
  cpp/core/ins.h    cpp/core/ins.cpp
  cpp/core/ekf.h    cpp/core/ekf.cpp
  cpp/core/gravity_model.h cpp/core/gravity_model.cpp
  cpp/core/terrain_provider.h cpp/core/terrain_provider.cpp
)
target_include_directories(stn_demo PRIVATE cpp/core)
target_link_libraries(stn_demo PRIVATE Eigen3::Eigen)

enable_testing()
add_executable(stn_tests cpp/tests/test_transforms.cpp)
add_test(NAME stn_tests COMMAND stn_tests)
