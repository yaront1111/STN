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
    cpp/core/gravity_model.cpp
    cpp/core/gravity_gradient_provider.cpp
    cpp/core/gravity_map_matcher.cpp
    cpp/core/config_reader.cpp
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