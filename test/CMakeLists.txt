# Test Suite CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

# Find GTest
find_package(GTest REQUIRED)
include(GoogleTest)

# Enable testing
enable_testing()

# Collect all test source files
set(TEST_SOURCES
    test_main.cpp

    # Core UKF tests (Priority 1 - Critical)
    core/ukf/test_sr_ukf.cpp
    core/ukf/test_augmented_ukf.cpp
    core/ukf/test_edge_cases.cpp
    core/ukf/test_state_vector.cpp
    core/ukf/test_strapdown.cpp
    core/ukf/test_barometer_integration.cpp

    # Performance tests
    performance/test_performance.cpp

    # Integration tests
    integration/test_integration.cpp
    integration/test_accuracy.cpp
    integration/test_reset_integration.cpp  # NEW: Reset integration tests
    # integration/test_real_data_accuracy.cpp  # TODO: Fix compilation

    # Core RBPF tests
    core/rbpf/test_rbpf.cpp
    core/rbpf/test_rbpf_enhanced.cpp  # NEW: Enhanced RBPF tests

    # Core integration tests
    core/test_hierarchical_filter.cpp
    core/test_hierarchical_filter_enhanced.cpp  # NEW: Enhanced hierarchical filter tests
    core/test_navigation_fixes.cpp  # NEW: Tests for navigation fixes

    # Utils tests (Priority 2)
    utils/test_math_utils.cpp
    utils/test_data_validator.cpp

    # Sensor tests (Priority 3)
    sensors/test_imu_reader.cpp
    sensors/test_barometer_reader.cpp
    sensors/test_sensor_manager.cpp

    # Map tests
    maps/test_map_manager.cpp
)

# Filter out non-existent files for now
set(EXISTING_TEST_SOURCES test_main.cpp)
foreach(TEST_FILE ${TEST_SOURCES})
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${TEST_FILE})
        list(APPEND EXISTING_TEST_SOURCES ${TEST_FILE})
    endif()
endforeach()

# Create test executable
add_executable(navigation_tests ${EXISTING_TEST_SOURCES})

# Link against the main project's source files (create a library first)
# We'll need to modify the main CMakeLists.txt to create a library
target_include_directories(navigation_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/core
    ${CMAKE_SOURCE_DIR}/src/core/ukf
    ${CMAKE_SOURCE_DIR}/src/core/rbpf
    ${CMAKE_SOURCE_DIR}/src/utils
    ${CMAKE_SOURCE_DIR}/src/sensors
    ${CMAKE_SOURCE_DIR}/src/maps
    ${CMAKE_SOURCE_DIR}/src/ml
)

# Link libraries
target_link_libraries(navigation_tests
    navigation_core  # Use the core library we created
    GTest::gtest
    GTest::gtest_main
    Eigen3::Eigen
    Threads::Threads
    ${YAMLCPP_LIBRARIES}
)

# Register tests with CTest
gtest_discover_tests(navigation_tests)

# Create graded test runner executable - dedicated accuracy validation
add_executable(graded_tests graded_tests.cpp)
target_include_directories(graded_tests PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/core
    ${CMAKE_SOURCE_DIR}/src/core/ukf
    ${CMAKE_SOURCE_DIR}/src/core/rbpf
    ${CMAKE_SOURCE_DIR}/src/utils
    ${CMAKE_SOURCE_DIR}/src/sensors
    ${CMAKE_SOURCE_DIR}/src/maps
    ${CMAKE_SOURCE_DIR}/src/ml
)
target_link_libraries(graded_tests
    navigation_core
    GTest::gtest
    Eigen3::Eigen
    Threads::Threads
    ${YAMLCPP_LIBRARIES}
)

# Add custom test targets
add_custom_target(test_ukf
    COMMAND navigation_tests --gtest_filter=*UKF*
    DEPENDS navigation_tests
    COMMENT "Running UKF tests"
)

add_custom_target(test_critical
    COMMAND navigation_tests --gtest_filter=*Critical*
    DEPENDS navigation_tests
    COMMENT "Running critical tests"
)

# Grade the navigation system
add_custom_target(grade
    COMMAND graded_tests
    DEPENDS graded_tests
    COMMENT "Running tests and grading navigation system"
)

# Coverage target (if using coverage tools)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_custom_target(coverage
        COMMAND navigation_tests
        COMMAND gcov ${CMAKE_BINARY_DIR}/CMakeFiles/navigation_tests.dir/src/*.gcno
        DEPENDS navigation_tests
        COMMENT "Generating coverage report"
    )
endif()