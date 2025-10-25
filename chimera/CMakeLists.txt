cmake_minimum_required(VERSION 3.16)
project(chimera VERSION 0.1.0 LANGUAGES CXX)

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -fPIC -fsanitize=address -fsanitize=undefined")

# Find dependencies
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GTSAM REQUIRED)
find_package(yaml-cpp REQUIRED)

# Optional: GTest for unit tests
find_package(GTest)

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/src
    ${GTSAM_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

#############################################
# chimera_core library (main estimation core)
#############################################
add_library(chimera_core STATIC
    # Core
    src/core/ukf.cpp
    src/core/smoother_wrapper.cpp
    src/core/contract_monitor.cpp

    # Factors (header-only with some .cpp implementations)
    src/factors/altimeter_range_factor.cpp
    src/factors/baro_factor.cpp
    src/factors/mag_yaw_factor.cpp

    # Processors
    src/processors/lidar_processor.cpp
    src/processors/tof_processor.cpp
    src/processors/baro_processor.cpp

    # Utils
    src/utils/telemetry.cpp
    src/utils/timing.cpp
    src/utils/sensor_helpers.cpp
    src/utils/data_loaders.cpp
    src/utils/calibration.cpp
    src/utils/schema_validator.cpp
)

target_link_libraries(chimera_core
    gtsam
    gtsam_unstable
    Eigen3::Eigen
    yaml-cpp
)

target_include_directories(chimera_core PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:src>
)

#############################################
# chimera_node_multi - Multi-sensor real data (ONLY production app)
#############################################
add_executable(chimera_node_multi
    apps/chimera_node_multi.cpp
)

target_link_libraries(chimera_node_multi
    chimera_core
    pthread
    boost_json
)

#############################################
# Unit tests (if GTest found)
#############################################
if(GTest_FOUND)
    enable_testing()

    add_executable(chimera_tests
        # Original tests
        tests/cpp/test_factors.cpp
        tests/cpp/test_multi_node.cpp
        tests/cpp/test_imu_units.cpp
        # New comprehensive test suite
        tests/cpp/test_utilities.cpp
        tests/cpp/test_data_loading.cpp
        tests/cpp/test_sensor_matching.cpp
        tests/cpp/test_unit_detection.cpp
        tests/cpp/test_depth_algorithms.cpp
        # Hardening tests
        tests/cpp/test_orientation_boot.cpp
        # Phase 1 robustness tests
        tests/cpp/test_stuck_detection.cpp
        tests/cpp/test_watchdogs.cpp
        tests/cpp/test_property_invariants.cpp
        # Phase 2 integration tests
        tests/cpp/test_calibration.cpp
        tests/cpp/test_schema_validation.cpp
    )

    target_link_libraries(chimera_tests
        chimera_core
        GTest::GTest
        GTest::Main
        pthread
        boost_json
    )

    include(GoogleTest)
    gtest_discover_tests(chimera_tests)
endif()

#############################################
# Installation
#############################################
install(TARGETS chimera_core chimera_node_multi
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY src/core src/processors src/factors src/utils
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY config/
    DESTINATION share/chimera/config
    FILES_MATCHING PATTERN "*.yaml"
)

#############################################
# Summary
#############################################
message(STATUS "")
message(STATUS "========================================")
message(STATUS "CHIMERA Forever UAV Navigation Stack")
message(STATUS "========================================")
message(STATUS "Build type:      ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard:    ${CMAKE_CXX_STANDARD}")
message(STATUS "GTSAM:           ${GTSAM_VERSION}")
message(STATUS "Eigen3:          ${EIGEN3_VERSION_STRING}")
message(STATUS "GTest:           ${GTest_FOUND}")
message(STATUS "pybind11:        ${pybind11_FOUND}")
message(STATUS "========================================")
message(STATUS "")
