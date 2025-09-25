# GPS-Free Navigation System - Production Build
## Version 2.0.0

## Overview
This production-ready GPS-free navigation system has been enhanced with comprehensive features for real-world deployment, testing, and accuracy validation.

## Key Features Implemented

### 1. Enhanced Navigation Executable (`navigation_system`)
- **Command-line interface** with extensive options
- **Multiple run modes**: normal, dry-run, validate-only, headless
- **Configurable parameters**: log levels, directories, output formats
- **Performance profiling** capabilities
- **Real-time status reporting**
- **Graceful shutdown handling**
- **Version and help information**

### 2. Graded Test System (`graded_tests`)
- **Dedicated accuracy validation tests**
- **50m position error requirement testing**
- **Sensor failure resilience testing**
- **Long-term drift assessment**
- **Automatic grade calculation**
- **Detailed test reports**

### 3. Build System
- **CMake build configuration** (primary method)
- **Makefile** for alternative builds
- **Shell script** (`build.sh`) for easy compilation
- Supports both Debug and Release modes
- Automatic dependency checking

## Quick Start

### Building the System

```bash
# Using build script (recommended)
./build.sh Release

# Using CMake directly
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

# Using Make
make all
```

### Running the Navigation System

```bash
# Basic run with config file
./build/navigation_system -c config.yaml

# Run with custom options
./build/navigation_system -c config.yaml -l DEBUG -p  # Debug logging with profiling
./build/navigation_system -c config.yaml -m 1000      # Run for 1000 iterations (10 seconds)
./build/navigation_system -c config.yaml --dry-run     # Validate config without running

# Get help
./build/navigation_system -h
./build/navigation_system -v  # Version info
```

### Running Accuracy Tests

```bash
# Run graded tests for accuracy validation
./build/test/graded_tests

# The test suite will:
# - Test 50m accuracy requirement over 10 minutes
# - Test accuracy during sensor failures
# - Assess long-term drift
# - Generate grade report
```

## Command-Line Options

| Option | Description | Example |
|--------|-------------|---------|
| `-h, --help` | Show help message | `./navigation_system -h` |
| `-v, --version` | Show version info | `./navigation_system -v` |
| `-c, --config` | Specify config file | `-c config.yaml` |
| `-l, --log-level` | Set log level (DEBUG/INFO/WARN/ERROR) | `-l DEBUG` |
| `-d, --log-dir` | Set log directory | `-d logs/` |
| `-n, --dry-run` | Validate config without running | `--dry-run` |
| `-p, --profile` | Enable performance profiling | `-p` |
| `-m, --max-iter` | Maximum iterations to process | `-m 6000` |
| `-f, --format` | Output format (csv/json) | `-f json` |
| `-q, --headless` | Run without console output | `-q` |
| `--validate` | Validate configuration only | `--validate` |

## System Architecture

The production system includes:

1. **Core Navigation**:
   - Square-Root UKF on SO(3) manifold
   - Rao-Blackwellized Particle Filter
   - Hierarchical filter architecture

2. **Sensor Fusion**:
   - IMU (100Hz)
   - Barometer (10Hz)
   - Magnetometer (10Hz)
   - Gradiometer (1Hz)

3. **Map Integration**:
   - XGM2019e gravity model
   - SRTM terrain data
   - Composite map manager

4. **Production Features**:
   - Comprehensive logging system
   - Performance monitoring
   - Data validation
   - Health monitoring
   - Graceful error handling

## Testing & Validation

### Accuracy Requirements
- **Target**: <50m position error after 30 minutes without GPS
- **Test Duration**: 10 minutes minimum
- **Test Scenarios**: Normal flight, with maneuvers, sensor failures

### Test Results
The graded test system evaluates:
- Maximum position error
- Mean position error
- Final position error
- Drift rate per minute
- Sensor failure resilience

Grade calculation based on test pass rate:
- A+ (95-100%): All requirements exceeded
- B+ (87-94%): Production ready
- Below 87%: Needs improvement

## File Structure

```
stn-v0.1/
├── navigation_system.cpp      # Enhanced main executable
├── test/
│   └── graded_tests.cpp      # Accuracy validation tests
├── build.sh                   # Build script
├── Makefile                   # Alternative build system
├── CMakeLists.txt            # CMake configuration
└── config.yaml               # System configuration
```

## Performance Monitoring

The system provides real-time performance metrics:
- Processing time per iteration (target: <10ms)
- UKF, RBPF, and ML processing times
- Budget violations tracking
- CPU and memory usage
- Missed deadline counting

## Logging

Multiple log files are generated:
- `system.log`: General system events
- `state.log`: Navigation state evolution
- `measurements.log`: Sensor data
- `covariance.log`: Uncertainty evolution
- `residuals.log`: Innovation/residuals
- `resets.log`: Position reset events
- `performance.log`: Timing metrics

## Production Deployment

For production use:

1. **Configuration**: Adjust `config.yaml` for your specific requirements
2. **Data Files**: Ensure sensor data and map files are available
3. **Performance**: Use Release build for optimal performance
4. **Monitoring**: Enable logging and profiling for system health monitoring
5. **Testing**: Run graded tests to validate accuracy before deployment

## Notes on Accuracy

The system is designed to achieve <50m position error. Current test results may vary based on:
- Availability of real sensor data
- Map data quality
- Initial state accuracy
- Sensor calibration

For production deployment, ensure:
- Real sensor data files are available
- Map data (XGM2019e, SRTM) is properly loaded
- System is properly calibrated
- Initial position is accurately set

## Support

For issues or questions, consult:
- System logs for debugging
- `--dry-run` mode for configuration validation
- `--validate` mode for component checking
- Performance logs for optimization