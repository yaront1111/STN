# Navigation System Runner - Usage Guide

## Overview
The `run_navigation_system` executable is a comprehensive testing and production runner for the Spacetime Navigation System (STN). It provides extensive debugging, logging, and control features through command-line flags.

## Building
```bash
mkdir -p build
cd build
cmake ..
make run_navigation_system
```

## Basic Usage

### Quick Start
```bash
# Run with default settings (300-second mountain flight simulation)
./run_navigation_system

# Run with debug output
./run_navigation_system --debug

# Run for 10 minutes with profiling
./run_navigation_system --duration 600 --profile
```

## Command-Line Options

| Flag | Description | Default |
|------|-------------|---------|
| `--mode <type>` | Operating mode: test, production, simulation | simulation |
| `--scenario <name>` | Test scenario to run | mountain |
| `--duration <seconds>` | How long to run the simulation | 300 |
| `--log-level <level>` | Logging verbosity: DEBUG, INFO, WARN, ERROR | INFO |
| `--log-dir <path>` | Directory for all log files | logs |
| `--debug` | Enable all debug output (sets log level to DEBUG) | false |
| `--profile` | Enable performance profiling | false |
| `--record` | Record all sensor and state data | false |
| `--initial-lat <deg>` | Initial latitude in degrees | 47.0 |
| `--initial-lon <deg>` | Initial longitude in degrees | 8.0 |
| `--initial-alt <m>` | Initial altitude in meters | 5000 |

### Feature Toggles
| Flag | Description |
|------|-------------|
| `--enable-gravity` | Enable gravity gradient measurements |
| `--disable-gravity` | Disable gravity gradient measurements |
| `--enable-terrain` | Enable terrain correlation |
| `--disable-terrain` | Disable terrain correlation |
| `--enable-magnetometer` | Enable magnetometer updates |
| `--disable-magnetometer` | Disable magnetometer updates |
| `--enable-barometer` | Enable barometric altitude |
| `--disable-barometer` | Disable barometric altitude |

## Logging Structure

The system creates organized logs in the following structure:

```
logs/
├── navigation/       # Main navigation logs with timestamps
├── sensors/          # Raw sensor data logs
├── profiling/        # Performance profiling data
├── recordings/       # Full state and sensor recordings
└── debug/           # Detailed debug output
```

### Log Files Created

1. **Navigation Log** (`logs/navigation_YYYYMMDD_HHMMSS.log`)
   - System events, errors, warnings
   - Position updates and error metrics
   - Update counts from various sensors

2. **Profile Data** (`logs/profiling/profile_YYYYMMDD_HHMMSS.csv`)
   - Function call counts and timings
   - Performance bottleneck analysis

3. **Recorded Data** (`logs/recordings/nav_data_YYYYMMDD_HHMMSS.csv`)
   - Complete state history
   - Position, velocity, attitude
   - Error metrics and covariances

4. **Sensor Data** (`logs/recordings/nav_data_YYYYMMDD_HHMMSS_sensors.csv`)
   - Raw IMU measurements
   - Gravity measurements
   - All sensor updates

## Usage Examples

### Debug Mode Testing
```bash
# Full debug with all features and recording
./run_navigation_system --debug --profile --record --duration 600

# Debug specific scenario
./run_navigation_system --scenario mountain --debug --log-level DEBUG
```

### Performance Testing
```bash
# Test without gravity to see drift
./run_navigation_system --disable-gravity --duration 300 --record

# Test with only IMU and magnetometer
./run_navigation_system --disable-gravity --disable-terrain --disable-barometer
```

### Production Mode
```bash
# Real location with specific coordinates
./run_navigation_system --mode production \
    --initial-lat 37.7749 \
    --initial-lon -122.4194 \
    --initial-alt 100 \
    --duration 3600
```

### Data Analysis
```bash
# Record full session for analysis
./run_navigation_system --record --profile --duration 1800

# The recorded files can be analyzed with:
# - logs/recordings/nav_data_*.csv for state evolution
# - logs/profiling/profile_*.csv for performance metrics
# - logs/navigation_*.log for system events
```

## Performance Grades

The system automatically evaluates navigation performance:

- **Grade A**: RMS position error < 100m (Excellent)
- **Grade B+**: RMS position error < 200m (Very Good)
- **Grade B**: RMS position error < 500m (Good)
- **Grade C**: RMS position error > 500m (Needs Improvement)

## Interpreting Output

### Real-time Console Output
```
t=60s | Pos err: 145m (avg 132m) | Updates: G=6 M=120 B=300
```
- `t`: Elapsed time
- `Pos err`: Current position error
- `avg`: Average position error
- `G`: Gravity updates count
- `M`: Magnetometer updates count
- `B`: Barometer updates count

### Final Summary
```
PERFORMANCE METRICS:
  Position RMS: 156.3 m
  Position Avg: 142.7 m
  Position Max: 198.2 m
  Velocity RMS: 2.4 m/s

*** GRADE B+: VERY GOOD (<200m RMS) ***
```

## Troubleshooting

### Common Issues

1. **High position errors**
   - Check if gravity measurements are enabled
   - Verify initial position is reasonable
   - Increase update rates with debug mode

2. **No sensor updates**
   - Ensure features are enabled with --enable flags
   - Check log files for initialization errors

3. **Performance issues**
   - Use --profile to identify bottlenecks
   - Reduce log level from DEBUG to INFO
   - Disable visualization if enabled

## Advanced Usage

### Custom Scenarios
The runner supports different test scenarios. Currently available:
- `mountain`: Mountain flight over Switzerland Alps

### Batch Testing
```bash
# Run multiple tests with different configurations
for duration in 100 200 300 600; do
    ./run_navigation_system --duration $duration --record \
        --log-dir logs/test_$duration
done
```

### Continuous Integration
```bash
# CI-friendly output with error codes
./run_navigation_system --duration 300 --log-level ERROR || exit 1
```

## Files Created by Previous Test Runs

The test files (`test_*.cpp`) were created for specific testing scenarios:

- `test_gps_denied_simple.cpp`: Basic GPS-denied navigation demo
- `test_grade_b_plus.cpp`: Grade B+ performance validation
- `test_scaled_ukf_gps_denied.cpp`: Scaled UKF testing
- `test_system_comprehensive.cpp`: Full system integration test

The new `run_navigation_system` consolidates all these tests with configurable options.