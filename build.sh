#!/bin/bash

# GPS-Free Navigation System Build Script
# Version 2.0.0
# Builds both navigation_system and graded_tests executables

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Build configuration
BUILD_TYPE=${1:-Release}
CLEAN_BUILD=${2:-false}

echo -e "${BLUE}=== GPS-Free Navigation System Build Script ===${NC}"
echo -e "${BLUE}Version 2.0.0${NC}"
echo "Build Type: $BUILD_TYPE"
echo ""

# Function to check dependencies
check_dependencies() {
    echo -e "${YELLOW}Checking dependencies...${NC}"

    # Check for required tools
    if ! command -v cmake &> /dev/null; then
        echo -e "${YELLOW}CMake not found. Falling back to direct compilation.${NC}"
        return 1
    fi

    if ! command -v clang++ &> /dev/null && ! command -v g++ &> /dev/null; then
        echo -e "${RED}No C++ compiler found. Please install clang++ or g++${NC}"
        exit 1
    fi

    # Detect compiler
    if command -v clang++ &> /dev/null; then
        CXX=clang++
    else
        CXX=g++
    fi

    echo "Using compiler: $CXX"

    # Check for Eigen3
    if [ -d "/opt/homebrew/include/eigen3" ]; then
        EIGEN_INCLUDE="-I/opt/homebrew/include/eigen3"
    elif [ -d "/usr/include/eigen3" ]; then
        EIGEN_INCLUDE="-I/usr/include/eigen3"
    elif [ -d "/usr/local/include/eigen3" ]; then
        EIGEN_INCLUDE="-I/usr/local/include/eigen3"
    else
        echo -e "${RED}Eigen3 not found. Please install: brew install eigen (macOS) or apt-get install libeigen3-dev (Linux)${NC}"
        exit 1
    fi

    # Check for yaml-cpp
    if [ -d "/opt/homebrew/include" ]; then
        YAML_INCLUDE="-I/opt/homebrew/include"
        YAML_LIB="-L/opt/homebrew/lib -lyaml-cpp"
    elif pkg-config --exists yaml-cpp 2>/dev/null; then
        YAML_INCLUDE=$(pkg-config --cflags yaml-cpp)
        YAML_LIB=$(pkg-config --libs yaml-cpp)
    else
        YAML_INCLUDE=""
        YAML_LIB="-lyaml-cpp"
    fi

    echo -e "${GREEN}Dependencies checked successfully${NC}"
    return 0
}

# CMake build function
build_with_cmake() {
    echo -e "${YELLOW}Building with CMake...${NC}"

    # Clean build if requested
    if [ "$CLEAN_BUILD" = "true" ]; then
        echo "Cleaning previous build..."
        rm -rf build
    fi

    # Create build directory
    mkdir -p build
    cd build

    # Configure
    echo -e "${YELLOW}Configuring...${NC}"
    cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DBUILD_TESTS=ON

    # Build
    echo -e "${YELLOW}Building...${NC}"
    make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 2)

    cd ..

    echo -e "${GREEN}CMake build completed successfully${NC}"
    echo -e "${GREEN}Executables created:${NC}"
    echo "  - build/navigation_system"
    echo "  - build/test/graded_tests"
    echo "  - build/test/navigation_tests"
}

# Direct compilation function
build_direct() {
    echo -e "${YELLOW}Building with direct compilation...${NC}"

    # Set compiler flags
    if [ "$BUILD_TYPE" = "Debug" ]; then
        FLAGS="-g -O0 -DDEBUG"
    else
        FLAGS="-O3 -DNDEBUG -march=native"
    fi

    FLAGS="$FLAGS -std=c++17 -Wall -Wextra -Wno-unused-parameter"

    # Create build directory
    mkdir -p build/obj
    mkdir -p build/test

    echo -e "${YELLOW}Compiling source files...${NC}"

    # List of source files
    SOURCE_FILES=(
        "src/core/hierarchical_filter.cpp"
        "src/core/ukf/sr_ukf.cpp"
        "src/core/ukf/strapdown.cpp"
        "src/core/rbpf/rbpf.cpp"
        "src/sensors/sensor_manager.cpp"
        "src/sensors/imu_reader.cpp"
        "src/sensors/barometer_reader.cpp"
        "src/sensors/magnetometer_reader.cpp"
        "src/sensors/gradiometer_reader.cpp"
        "src/maps/map_manager.cpp"
        "src/maps/composite_map_manager.cpp"
        "src/maps/xgm2019e_map.cpp"
        "src/maps/srtm_terrain.cpp"
        "src/maps/map_interpolator.cpp"
        "src/utils/logger.cpp"
        "src/utils/data_validator.cpp"
        "src/utils/performance_monitor.cpp"
        "src/utils/math_utils.cpp"
        "src/utils/trajectory_analyzer.cpp"
        "src/ml/onnx_predictor.cpp"
    )

    # Compile each source file
    for src in "${SOURCE_FILES[@]}"; do
        if [ -f "$src" ]; then
            obj_name=$(basename ${src%.cpp}.o)
            echo "  Compiling $src..."
            $CXX -c "$src" $FLAGS -I. $EIGEN_INCLUDE $YAML_INCLUDE -o "build/obj/$obj_name"
        else
            echo -e "${YELLOW}  Warning: $src not found, skipping${NC}"
        fi
    done

    # Link navigation_system executable
    echo -e "${YELLOW}Linking navigation_system...${NC}"
    $CXX $FLAGS navigation_system.cpp build/obj/*.o $YAML_LIB -lpthread -o build/navigation_system

    # Build graded_tests if GTest is available
    if pkg-config --exists gtest 2>/dev/null || [ -f "/usr/local/lib/libgtest.a" ] || [ -f "/opt/homebrew/lib/libgtest.a" ]; then
        echo -e "${YELLOW}Building graded_tests...${NC}"

        # Find GTest
        if pkg-config --exists gtest 2>/dev/null; then
            GTEST_FLAGS=$(pkg-config --cflags gtest)
            GTEST_LIBS=$(pkg-config --libs gtest)
        elif [ -f "/opt/homebrew/lib/libgtest.a" ]; then
            GTEST_FLAGS="-I/opt/homebrew/include"
            GTEST_LIBS="-L/opt/homebrew/lib -lgtest -lgtest_main"
        else
            GTEST_FLAGS="-I/usr/local/include"
            GTEST_LIBS="-L/usr/local/lib -lgtest -lgtest_main"
        fi

        $CXX $FLAGS test/graded_tests.cpp build/obj/*.o \
            -I. $EIGEN_INCLUDE $YAML_INCLUDE $GTEST_FLAGS \
            $YAML_LIB $GTEST_LIBS -lpthread \
            -o build/test/graded_tests

        echo -e "${GREEN}graded_tests built successfully${NC}"
    else
        echo -e "${YELLOW}GTest not found. Skipping graded_tests build.${NC}"
        echo "To build tests, install GTest: brew install googletest (macOS) or apt-get install libgtest-dev (Linux)"
    fi

    echo -e "${GREEN}Direct compilation completed successfully${NC}"
    echo -e "${GREEN}Executables created:${NC}"
    echo "  - build/navigation_system"
    if [ -f "build/test/graded_tests" ]; then
        echo "  - build/test/graded_tests"
    fi
}

# Main build logic
echo ""

if check_dependencies; then
    if command -v cmake &> /dev/null; then
        build_with_cmake
    else
        build_direct
    fi
else
    build_direct
fi

echo ""
echo -e "${GREEN}=== Build Complete ===${NC}"
echo ""
echo "To run the navigation system:"
echo "  ./build/navigation_system -c config.yaml"
echo "  ./build/navigation_system -h  # for help"
echo ""
echo "To run accuracy tests and see grade:"
echo "  ./build/test/graded_tests"
echo ""
echo "Quick test (10 seconds):"
echo "  ./build/navigation_system -c config.yaml -m 1000 --profile"
echo ""