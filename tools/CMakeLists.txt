# Tools and utilities for AION

# Add tools as they become available
add_subdirectory(replay)
# add_subdirectory(metrics)

# STN test program
add_executable(test_stn test_stn.cpp)
target_link_libraries(test_stn PRIVATE
    measurements
    aion_core
    Eigen3::Eigen
    yaml-cpp
)