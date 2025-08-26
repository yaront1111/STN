cmake_minimum_required(VERSION 3.10)
project(STN)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Options
option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_BENCHMARKS "Build performance benchmarks" OFF)

# Find Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Core library sources
set(CORE_SOURCES
    cpp/core/ins.cpp
    cpp/core/ekf.cpp
    cpp/core/gravity_model.cpp
    cpp/core/radalt_sampler.cpp
    cpp/core/terrain_provider.cpp
)

# Create core library for reuse
add_library(stn_core STATIC ${CORE_SOURCES})
target_include_directories(stn_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/core
    ${CMAKE_CURRENT_SOURCE_DIR}/core
)
target_link_libraries(stn_core Eigen3::Eigen)

# Main executable
add_executable(stn_demo core/main.cpp)
target_link_libraries(stn_demo stn_core)

# Compiler flags for optimization
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(stn_demo PRIVATE -O3 -march=native)
    target_compile_options(stn_core PRIVATE -O3 -march=native)
endif()

# Unit tests
if(BUILD_TESTS)
    enable_testing()
    
    # Download and configure Google Test
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG release-1.12.1
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
    
    # Test sources
    set(TEST_SOURCES
        tests/test_main.cpp
        tests/test_quaternion.cpp
        tests/test_gravity.cpp
        tests/test_ekf.cpp
        tests/test_terrain.cpp
    )
    
    # Test executable
    add_executable(stn_tests ${TEST_SOURCES})
    target_link_libraries(stn_tests
        stn_core
        gtest_main
        gmock
    )
    
    # Add tests to CTest
    include(GoogleTest)
    gtest_discover_tests(stn_tests)
endif()

# Benchmarks
if(BUILD_BENCHMARKS)
    add_executable(stn_benchmark
        benchmarks/bench_main.cpp
    )
    target_link_libraries(stn_benchmark stn_core)
endif()