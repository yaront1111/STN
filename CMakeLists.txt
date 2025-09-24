cmake_minimum_required(VERSION 3.16)
project(GPSFreeNavigation VERSION 2.0.0 LANGUAGES CXX)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native -mtune=native")

# Add safety and warning flags
add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Werror
    -Wno-unused-parameter
    -Wno-unused-variable
)

# Thread sanitizer for debug builds
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-fsanitize=thread -fno-omit-frame-pointer)
    add_link_options(-fsanitize=thread)
endif()

# Find required packages
find_package(Threads REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Find yaml-cpp using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)

# Find ONNX Runtime (optional for ML)
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
    HINTS /usr/local/include/onnxruntime
          /usr/include/onnxruntime
)
find_library(ONNXRUNTIME_LIB onnxruntime
    HINTS /usr/local/lib
          /usr/lib
)

if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIB)
    message(STATUS "Found ONNX Runtime")
    set(HAVE_ONNX TRUE)
    add_definitions(-DHAVE_ONNX)
else()
    message(WARNING "ONNX Runtime not found - ML features disabled")
    set(HAVE_ONNX FALSE)
endif()

# Source files
set(SOURCES
    navigation_system.cpp

    # Core navigation
    src/core/hierarchical_filter.cpp
    src/core/ukf/sr_ukf.cpp
    src/core/ukf/strapdown.cpp
    src/core/rbpf/rbpf.cpp

    # Sensors
    src/sensors/sensor_manager.cpp
    src/sensors/imu_reader.cpp
    src/sensors/barometer_reader.cpp
    src/sensors/magnetometer_reader.cpp
    src/sensors/gradiometer_reader.cpp

    # Maps
    src/maps/map_manager.cpp
    src/maps/xgm2019e_map.cpp
    src/maps/srtm_terrain.cpp
    src/maps/map_interpolator.cpp

    # Utilities
    src/utils/logger.cpp
    src/utils/data_validator.cpp
    src/utils/performance_monitor.cpp
    src/utils/math_utils.cpp

    # Debug/Analysis
    src/utils/trajectory_analyzer.cpp
)

# Add ML sources if ONNX is available
if(HAVE_ONNX)
    list(APPEND SOURCES
        src/ml/onnx_predictor.cpp
        src/ml/ood_detector.cpp
    )
endif()

# Create executable
add_executable(navigation_system ${SOURCES})

# Include directories
target_include_directories(navigation_system PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIRS}
    ${YAMLCPP_INCLUDE_DIRS}
)

if(HAVE_ONNX)
    target_include_directories(navigation_system PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
endif()

# Add library directories
link_directories(${YAMLCPP_LIBRARY_DIRS})

# Link libraries
target_link_libraries(navigation_system
    Threads::Threads
    Eigen3::Eigen
    ${YAMLCPP_LIBRARIES}
    stdc++fs  # For filesystem
)

if(HAVE_ONNX)
    target_link_libraries(navigation_system ${ONNXRUNTIME_LIB})
endif()

# Math library
target_link_libraries(navigation_system m)

# Install targets
install(TARGETS navigation_system
    RUNTIME DESTINATION bin
)

# Install configuration
install(FILES config.yaml
    DESTINATION etc/gps_free_nav
)

# Create necessary directories
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/logs)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/data)

# Copy test data if available
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/data/flight)
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/flight
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/data
    )
endif()

# Print configuration summary
message(STATUS "")
message(STATUS "GPS-Free Navigation System Configuration:")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "  ONNX Runtime: ${HAVE_ONNX}")
message(STATUS "")

# Add custom target for running the system
add_custom_target(run
    COMMAND navigation_system config.yaml
    DEPENDS navigation_system
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Running GPS-Free Navigation System"
)