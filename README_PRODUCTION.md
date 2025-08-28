# STN Navigator - Production Real-World Navigation System

## Overview

This is a production-grade Strapdown Terrain-referenced Navigation (STN) system that uses:
- **Real SRTM terrain data** for terrain-referenced navigation
- **Real EGM2008 gravity model** for gravity anomaly corrections  
- **Real flight data** from actual IMU and radar altimeter sensors

**This is NOT a simulation** - this system is designed for real-world navigation using actual sensor data and real terrain databases.

## System Requirements

### Required Data Files

1. **SRTM Terrain Data** (`.hgt` files)
   - Place in: `data/terrain/`
   - Format: SRTM3 (3 arc-second resolution)
   - Example: `N47E008.hgt` for Zurich area
   - Download from: NASA Earthdata or USGS

2. **EGM2008 Gravity Model** (optional but recommended)
   - Place in: `data/egm2008.dat`
   - Provides gravity anomaly corrections
   - Download from: NGA Office of Geomatics

3. **Flight Data**
   - IMU data: `t, ax, ay, az, gx, gy, gz` (m/s², rad/s)
   - Radar altimeter: `t, agl` (seconds, meters)
   - Place in: `data/flight/`

## Building the System

```bash
mkdir build
cd build
cmake ..
make -j8
```

This creates the production executable: `build/stn_navigator`

## Data Preparation

### Preparing Real Flight Data

```bash
python3 python/production/prepare_flight_data.py \
    <imu_file> <initial_lat> <initial_lon> <initial_alt_msl>

# Example for Zurich:
python3 python/production/prepare_flight_data.py \
    flight_imu.csv 47.4 8.5 1500
```

This will:
- Load your IMU data
- Query SRTM terrain elevations
- Generate radar altimeter estimates (if not available)
- Create navigation configuration

## Running Navigation

```bash
./build/stn_navigator <radalt_file> <imu_file> <output_file>

# Example:
./build/stn_navigator \
    data/flight/radalt.csv \
    data/flight/imu.csv \
    data/flight/nav_output.csv
```

## Configuration

Edit `config/stn_realworld.cfg` for production parameters:

```ini
# TRN Parameters
trn.alpha_base = 0.15       # Learning rate
trn.slope_threshold = 0.002  # Minimum terrain slope
trn.nis_gate = 12.59        # Outlier rejection gate

# EKF Parameters  
ekf.q_pos = 0.5             # Position process noise
ekf.r_trn_base = 25.0       # TRN measurement noise
```

## Performance Metrics

The system targets:
- **CEP95**: ≤15 meters (95th percentile circular error)
- **Stability**: ≥85% (measurements within 30m)
- **Update Rate**: 100-200 Hz IMU, 1-2 Hz TRN

## Production Features

- **Real SRTM Terrain**: No synthetic terrain models
- **Real Gravity Model**: EGM2008 or measured anomalies
- **Adaptive TRN**: Time-varying confidence with convergence
- **Robust Outlier Rejection**: NIS gating and Huber estimation
- **15-State EKF**: Full observability including IMU biases

## Directory Structure

```
stn-v0.1/
├── build/              # Build directory
│   └── stn_navigator   # Production executable
├── config/             # Configuration files
│   └── stn_realworld.cfg
├── data/               
│   ├── terrain/        # SRTM .hgt files
│   ├── flight/         # Flight data (IMU, radar)
│   └── egm2008.dat     # Gravity model
├── cpp/core/           # C++ navigation core
└── python/production/  # Production data tools
    ├── srtm_reader.py
    ├── flight_data_loader.py
    └── prepare_flight_data.py
```

## Troubleshooting

### "Failed to open SRTM tile"
- Download the required SRTM tile for your flight area
- Place .hgt file in `data/terrain/`

### "ERROR: EGM2008 file required"
- System works without it but accuracy improves with gravity corrections
- Download EGM2008 data or disable gravity updates in config

### High NIS / TRN Rejection
- Check terrain slope (needs >0.2% grade)
- Verify radar altimeter calibration
- Check IMU bias estimation

## License

Production system - contact for licensing information.

## Support

For production support, contact the STN development team.