# Gravity Navigator

**Pure Gravity-Based Navigation System with Map Matching**

A GPS-denied navigation system that achieves sub-100m accuracy using only gravity gradients, gravity anomaly map matching, and inertial measurements.

## Overview

This system implements cutting-edge gravity-based navigation without relying on GPS or external radio signals. It uses:

- **Gravity Gradient Measurements**: Continuous weak observability from gravity tensor
- **Gravity Anomaly Map Matching**: Periodic absolute position fixes (like TERCOM but for gravity)
- **Unscented Kalman Filter**: Stable error-state formulation for sensor fusion
- **Dynamic Maneuvers**: S-turns and other patterns to enhance observability

## Key Features

- **GPS-Independent**: No external signals required
- **Passive Operation**: Undetectable, unjammable
- **Map Matching**: Correlates gravity signatures with EGM2008 database
- **Sub-100m Accuracy**: With proper tuning and map matching
- **Real-Time Performance**: Optimized C++ implementation
- **Hardware Ready**: Interfaces for real IMU, gradiometer, and other sensors

## Architecture

```
IMU (100Hz) → UKF Predict
     ↓
Gravity Gradients → Continuous Updates
     ↓
Record Gravity Signature (30-60 measurements)
     ↓  
Every 30 seconds: CORRELATE WITH MAP
     ↓
High Confidence Match → POSITION FIX
     ↓
Reset Accumulated Drift → Continue
```

## Building

### Requirements

- C++17 compiler
- CMake 3.14+
- Eigen3 library
- POSIX threads

### Compilation

```bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

## Usage

### Basic Operation

```bash
./gravity_navigator [lat] [lon] [alt]
```

Example:
```bash
./gravity_navigator 47.0 8.0 1000  # Start at 47°N, 8°E, 1000m altitude
```

### Configuration

Edit `config/gravity_primary.yaml` to tune:
- UKF parameters
- Process noise levels
- Sensor noise models
- Map matching thresholds
- Hardware interfaces

### Output

The navigator outputs CSV files with:
- Position (lat, lon, alt)
- Velocity (north, east, down)
- Attitude (roll, pitch, yaw)
- Sensor update counts
- Error estimates

## Core Components

### UKF (Unscented Kalman Filter)
- **File**: `cpp/core/ukf.cpp`
- **Features**: 
  - Error-state formulation (15D error, 16D full state)
  - Handles quaternion singularities
  - Multiple sensor fusion methods

### Gravity Map Matcher
- **File**: `cpp/core/gravity_map_matcher.cpp`
- **Features**:
  - Records gravity signatures
  - Correlates with EGM2008 model
  - Provides absolute position fixes
  - Configurable search radius and resolution

### Gravity Gradient Provider
- **File**: `cpp/core/gravity_gradient_provider.cpp`
- **Features**:
  - Computes gravity tensor from EGM2008
  - Spherical harmonic evaluation
  - Real-time performance

## Data Requirements

### EGM2008 Gravity Model
Download the gravity model coefficients:
```bash
cd data
python3 download_real_egm2008.py
```

### SRTM Terrain Data (Optional)
For terrain correlation:
```bash
cd data
python3 download_real_srtm.py
```

## Performance

With proper configuration:
- **Position Drift**: <1km over 30 seconds between fixes
- **After Map Match**: Reset to <50m accuracy
- **Heading Accuracy**: <5° with magnetometer constraint
- **Computational Load**: <10% CPU on modern processors

## Hardware Integration

The system includes interfaces for:
- **IMU**: VectorNav VN-200 or similar
- **Gradiometer**: When available
- **Magnetometer**: For heading constraint
- **Barometer**: For altitude aiding
- **Radar Altimeter**: For terrain correlation

## Analysis Tools

Analyze navigation results:
```bash
python3 analyze_results.py data/gravity_nav.csv
```

## Testing

Run validation tests (if built):
```bash
./test_gravity_map_match  # Test map matching
./test_gps_denied_tuned   # Test GPS-denied navigation
```

## License

Proprietary - All rights reserved

## Contact

For questions about gravity-based navigation:
[Contact Information]

---

*This system demonstrates that high-accuracy navigation is possible using only the Earth's gravity field - no GPS required.*