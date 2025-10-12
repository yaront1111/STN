cmake_minimum_required(VERSION 3.22)
project(aion VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Options
option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_TOOLS "Build utility tools" ON)
option(BUILD_DOCS "Build documentation" OFF)
option(USE_CUDA "Enable CUDA acceleration" OFF)
option(USE_ROS2 "Enable ROS2 interface" OFF)

# Include custom cmake modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(cmake/CommonFlags.cmake)

# Find required packages
find_package(Eigen3 3.4 REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG)

# Optional packages
find_package(GTSAM QUIET)
find_package(OpenCV QUIET)
find_package(Sophus QUIET)
find_package(absl REQUIRED)

if(USE_CUDA)
    enable_language(CUDA)
    find_package(CUDA REQUIRED)
    find_package(CUDNN)
endif()

# Global include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${EIGEN3_INCLUDE_DIRS}
)

# Add subdirectories
add_subdirectory(msgs)
add_subdirectory(core)
add_subdirectory(terrain)
add_subdirectory(measurements)
add_subdirectory(frontends)
add_subdirectory(sensors)
add_subdirectory(apps)

if(BUILD_TOOLS)
    add_subdirectory(tools)
endif()

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Install configuration
install(DIRECTORY configs/ DESTINATION ${CMAKE_INSTALL_PREFIX}/configs)
install(DIRECTORY scripts/ DESTINATION ${CMAKE_INSTALL_PREFIX}/scripts
    PATTERN "*.py" PERMISSIONS OWNER_EXECUTE GROUP_EXECUTE WORLD_READ)

# Package configuration
include(CPack)
set(CPACK_PACKAGE_NAME "aion")
set(CPACK_PACKAGE_VENDOR "AION Navigation")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "All-Weather Integrated Opportunistic Navigation")
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})