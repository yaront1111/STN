# CHIMERA Project - Claude Code Instructions

## Project Overview

**CHIMERA Forever** - Multi-sensor fusion navigation system for UAVs using GTSAM factor graphs.

### Architecture
- **Core**: UKF + ISAM2 smoother (hybrid filter-smoother)
- **Sensors**: IMU (400Hz), Magnetometer (20Hz), LiDAR/Baro (10Hz), Optical Flow, Airspeed
- **Factors**: IMU preintegration, Mag yaw, Baro altitude, OF velocity, Airspeed+wind

### Key Conventions
1. **Units**: Always verify sensor units (gyro in rad/s, accel in m/s², airspeed in m/s)
2. **Frames**: NWU (world) ↔ FRD (body) - apply frame flips consistently
3. **Coordinates**: Z-up world frame, altitude = positive up
4. **Time**: All sensors timestamped, use `findNearest()` with tolerance

## Code Style Preferences

### When Making Changes
- **Always read first**: Use Read tool before Edit (required by tool policy)
- **Minimal edits**: Prefer Edit over Write for existing files
- **Test after changes**: Build + run tests to verify
- **Unit tests**: Add tests for critical fixes (see tests/cpp/)
- **Comments**: Explain WHY, not WHAT (code should be self-documenting)

### Sensor Unit Checking
- IMU: Check gyro (rad/s vs deg/s), accel (m/s² vs g)
- Mag: Verify frame alignment with IMU (apply same flip)
- Optical Flow: Check px/s vs px/frame (divide by dt if needed)
- Airspeed: Verify m/s (not mph, knots, or km/h)

### Debugging Philosophy
1. **Isolate**: Disable sensors one-by-one to find culprit
2. **Units first**: 90% of bugs are unit mismatches or frame flips
3. **Visualize**: Print first few samples, check ratios (gyro/mag, etc.)
4. **Gate values**: Reject outliers before factors

## Common Tasks

### Building
```bash
cmake --build build --target chimera_node_multi -j4
```

### Running Tests
```bash
./build/chimera_tests --gtest_filter="TestName.*"
```

### Checking Drift
```bash
./build/chimera_node_multi 2>&1 | grep -E "^(\[t=|    \[BIAS\])" | tail -20
```

## Known Issues

### Sensors
- **LiDAR**: Stuck at 0.5m in current dataset (correctly rejected)
- **Gyro bias**: ~67-503 deg/s (data may be in deg/s, needs investigation)
- **OF**: Weak constraint, altitude-dependent

### Fixes Applied
- Dangling lambda fix (gyro conversion)
- Mag tilt-comp yaw (33× ratio mismatch fixed)
- Velocity prior only when unconstrained
- Baro enabled early (σ=0.35m for t<8s)
- Wind prior loose early (σ=2 m/s for k<20)

## File Locations

### Core Fusion
- `apps/chimera_node_multi.cpp` - Main multi-sensor node
- `src/core/ukf.cpp` - UKF implementation
- `src/core/smoother_wrapper.cpp` - ISAM2 wrapper

### Factors (header-only)
- `include/chimera/factors/mag_yaw_factor.h`
- `include/chimera/factors/baro_factor.h`
- `include/chimera/factors/altimeter_range_factor.h`
- `include/chimera/factors/optical_flow_factor.h`
- `include/chimera/factors/aero_velocity_factor.h`

### Tests
- `tests/cpp/test_imu_units.cpp` - IMU unit/frame tests
- `tests/cpp/test_factors.cpp` - Factor Jacobian tests
- `tests/cpp/test_multi_node.cpp` - Integration tests

### Data
- `DATA/flight_all_sensors_complete.json` - Full sensor suite (60s)
- `DATA/optical_flow.json` - OF measurements

## Preferences for This Project

1. **Always use unit tests** when fixing critical issues (gyro, mag, OF)
2. **Avoid emojis** in code comments or output (professional codebase)
3. **Reference locations** using `file:line` format (e.g., `ukf.cpp:219`)
4. **Debug outputs** should be concise, < 80 chars per line
5. **Commit messages** should reference issue being fixed

## When Stuck

1. Check sensor units (print raw values, compute ratios)
2. Verify frame alignment (NWU vs FRD, check signs)
3. Isolate factor (disable one-by-one)
4. Compare to working baseline (test_multi_node.cpp)
5. Ask for ground truth data or expected values
