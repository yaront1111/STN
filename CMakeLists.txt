cmake_minimum_required(VERSION 3.14)
project(GravityNavigator VERSION 2.0.0)

# GRAVITY-PRIMARY NAVIGATION SYSTEM
# Pure gravity-based navigation with map matching

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native")

# Find required packages
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)

# Core gravity navigation library
add_library(gravity_core STATIC
    cpp/core/types.cpp
    cpp/core/ukf.cpp
    cpp/core/ukf_config.cpp
    cpp/core/ukf_sigma_points.cpp
    cpp/core/ukf_measurements.cpp
    cpp/core/ukf_math_utils.cpp
    cpp/core/ukf_state_scaler_v2.cpp
    cpp/core/ukf_scaled.cpp
    cpp/core/gravity_gradient_provider.cpp
    cpp/core/gravity_map_matcher.cpp
    cpp/core/hierarchical_map_matcher.cpp
    cpp/core/config_reader.cpp
    cpp/core/terrain_correlator.cpp
    cpp/core/srtm_provider.cpp
    cpp/core/particle_filter.cpp
    cpp/core/adaptive_filter.cpp
)

target_include_directories(gravity_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/core
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/hardware
)

target_link_libraries(gravity_core 
    Eigen3::Eigen
    Threads::Threads
)

# Main gravity navigator executable
add_executable(gravity_navigator
    core/gravity_navigator.cpp
)

target_link_libraries(gravity_navigator
    gravity_core
)

# Gravity map matching test
add_executable(test_gravity_map_match
    tests/test_gravity_map_match.cpp
)

target_link_libraries(test_gravity_map_match
    gravity_core
)

# Test executables - commented out until test files are created
# add_executable(test_two_factor test_two_factor.cpp)
# target_link_libraries(test_two_factor gravity_core)

# add_executable(test_system_comprehensive test_system_comprehensive.cpp)
# target_link_libraries(test_system_comprehensive gravity_core)

# add_executable(test_scaled_ukf_gps_denied test_scaled_ukf_gps_denied.cpp)
# target_link_libraries(test_scaled_ukf_gps_denied gravity_core)

# add_executable(test_gps_denied_simple test_gps_denied_simple.cpp)
# target_link_libraries(test_gps_denied_simple gravity_core)

# add_executable(test_grade_b_plus test_grade_b_plus.cpp)
# target_link_libraries(test_grade_b_plus gravity_core)

# Comprehensive navigation runner with debug and logging
add_executable(run_navigation_system
    run_navigation_system.cpp
)

target_link_libraries(run_navigation_system
    gravity_core
)


# Installation
install(TARGETS gravity_navigator
    RUNTIME DESTINATION bin
)

install(FILES config/gravity_primary.yaml
    DESTINATION etc/gravity_navigator
)

# Print configuration
message(STATUS "")
message(STATUS "====================================")
message(STATUS "  GRAVITY NAVIGATOR v${PROJECT_VERSION}")
message(STATUS "====================================")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "====================================")
message(STATUS "")