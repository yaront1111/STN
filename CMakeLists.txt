cmake_minimum_required(VERSION 3.14)
project(GravitySTN VERSION 2.0.0)

# GRAVITY-PRIMARY NAVIGATION SYSTEM
# All TRN code has been removed

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wextra")

# Find required packages
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)

# Optional packages
find_package(GTSAM QUIET)
find_package(OpenMP QUIET)

if(OpenMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    message(STATUS "OpenMP found - parallel processing enabled")
endif()

# Core gravity navigation library
add_library(gravity_core STATIC
    cpp/core/types.cpp
    cpp/core/ukf.cpp
    cpp/core/ukf_stable.cpp
    cpp/core/gravity_model.cpp
    cpp/core/gravity_gradient_provider.cpp
)

target_include_directories(gravity_core PUBLIC
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

# Hardware interface library (optional)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cpp/hardware/hardware_interface.cpp")
    add_library(hardware_interface STATIC
        cpp/hardware/hardware_interface.cpp
        cpp/hardware/vectornav_interface.cpp
    )
    target_link_libraries(hardware_interface gravity_core)
    target_link_libraries(gravity_navigator hardware_interface)
endif()

# Factor graph optimization (optional)
if(GTSAM_FOUND)
    add_definitions(-DUSE_GTSAM)
    add_library(factor_graph STATIC
        cpp/core/factor_graph_optimizer.cpp
    )
    target_link_libraries(factor_graph 
        gravity_core
        gtsam
    )
    target_link_libraries(gravity_navigator factor_graph)
    message(STATUS "GTSAM found - factor graph optimization enabled")
endif()

# Test program with real data
add_executable(test_real_data
    core/test_with_real_data.cpp
)

target_link_libraries(test_real_data
    gravity_core
    Eigen3::Eigen
)

# Tests
enable_testing()
add_test(NAME real_data_test COMMAND test_real_data flight_data.csv)

# Test stable UKF
add_executable(test_ukf_stable
    cpp/tests/test_ukf_stable.cpp
)

target_link_libraries(test_ukf_stable
    gravity_core
    Eigen3::Eigen
)

add_test(NAME ukf_stable_test COMMAND test_ukf_stable)

# Benchmarks and tools - to be created later

# Installation
install(TARGETS gravity_navigator
    RUNTIME DESTINATION bin
)

install(DIRECTORY config/
    DESTINATION share/gravity_stn/config
)

install(FILES 
    README_GRAVITY.md
    DESTINATION share/gravity_stn
)

# Systemd service - to be created later

# Package configuration
set(CPACK_GENERATOR "DEB;RPM;TGZ")
set(CPACK_PACKAGE_NAME "gravity-stn")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION "Gravity-Primary Spacetime Navigation System")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libeigen3-dev (>= 3.3)")
set(CPACK_RPM_PACKAGE_REQUIRES "eigen3-devel >= 3.3")
include(CPack)

# Print configuration
message(STATUS "")
message(STATUS "====================================")
message(STATUS "  GRAVITY-PRIMARY STN v${PROJECT_VERSION}")
message(STATUS "====================================")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  OpenMP: ${OpenMP_FOUND}")
message(STATUS "  GTSAM: ${GTSAM_FOUND}")
message(STATUS "====================================")
message(STATUS "  TRN Status: COMPLETELY REMOVED")
message(STATUS "====================================")
message(STATUS "")