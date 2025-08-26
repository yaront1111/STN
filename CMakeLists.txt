cmake_minimum_required(VERSION 3.10)
project(STN)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Collect source files
set(SOURCES
    core/main.cpp
    cpp/core/ins.cpp
    cpp/core/ekf.cpp
    cpp/core/gravity_model.cpp
    cpp/core/radalt_sampler.cpp
)

# Create executable
add_executable(stn_demo ${SOURCES})

# Include directories
target_include_directories(stn_demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/core
    ${CMAKE_CURRENT_SOURCE_DIR}/core
)

# Link Eigen
target_link_libraries(stn_demo Eigen3::Eigen)

# Compiler flags for optimization
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(stn_demo PRIVATE -O3 -march=native)
endif()