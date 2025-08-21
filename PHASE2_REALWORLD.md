# Phase 2: Real-World Data Integration

## Objective
Transition STN from simulation to real-world operation by integrating actual sensor data, terrain maps, and gravity models.

## Week 1: Data Acquisition

### 1.1 Public Flight Datasets

**Option A: KAIST Urban Dataset**
- **Pros**: High-quality UAV data, urban environment, IMU + GPS + Baro
- **Link**: https://irap.kaist.ac.kr/dataset/
- **Format**: ROS bags, CSV exports available
- **Coverage**: Seoul, South Korea

**Option B: EuRoC MAV Dataset**
- **Pros**: Micro aerial vehicle, indoor/outdoor, ground truth from Vicon
- **Link**: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
- **Format**: ASL format, IMU at 200Hz

**Option C: KITTI Dataset**
- **Pros**: Automotive but excellent sensor suite, large community
- **Link**: http://www.cvlibs.net/datasets/kitti/
- **Format**: Well-documented, includes IMU/GPS/Velodyne

### 1.2 Terrain Data Sources

**SRTM (Shuttle Radar Topography Mission)**
- Resolution: 30m globally, 1 arc-second
- Coverage: 60°N to 56°S
- Access: https://earthexplorer.usgs.gov/

**ASTER GDEM**
- Resolution: 30m global
- Coverage: 83°N to 83°S
- Access: https://asterweb.jpl.nasa.gov/gdem.asp

**High-Resolution (USA Only)**
- USGS 3DEP: 1m-10m resolution
- Access: https://www.usgs.gov/3d-elevation-program

### 1.3 Gravity Models

**EGM2008**
- Resolution: 2.5 arc-minute grid
- Includes gravity anomalies
- Download: https://earth-info.nga.mil/index.php?dir=wgs84&action=wgs84#tab_egm2008

**XGM2019**
- Latest high-resolution model
- Improved accuracy over EGM2008
- Access: http://icgem.gfz-potsdam.de/

## Week 2: Data Pipeline Implementation

### 2.1 Dataset Parser
```cpp
// data_parser.h
class RealDataParser {
public:
    struct SensorPacket {
        double timestamp;
        Eigen::Vector3d accel;  // m/s^2
        Eigen::Vector3d gyro;   // rad/s
        double pressure;        // Pa
        bool has_gps;
        Eigen::Vector3d gps_lla; // lat, lon, alt
    };
    
    bool loadDataset(const std::string& path);
    SensorPacket getNextPacket();
};
```

### 2.2 Terrain Interface
```cpp
// terrain_real.h
class RealTerrainProvider {
private:
    std::vector<float> dem_data;
    double lat_min, lat_max, lon_min, lon_max;
    int rows, cols;
    
public:
    bool loadDEM(const std::string& geotiff_path);
    double getElevation(double lat, double lon);
    Eigen::Vector2d getGradient(double lat, double lon);
};
```

### 2.3 Gravity Interface
```cpp
// gravity_real.h
class EGM2008Provider {
private:
    std::vector<float> gravity_grid;
    
public:
    bool loadEGM2008(const std::string& data_path);
    double getGravityAnomaly(double lat, double lon, double alt);
};
```

## Week 3: Integration & Validation

### 3.1 Synchronization Module
- Align IMU, GPS, and baro timestamps
- Handle different sampling rates
- Interpolate missing data

### 3.2 Coordinate Transformations
- WGS84 ↔ ECEF ↔ NED conversions
- UTM projections for local tangent plane
- Geoid height corrections

### 3.3 Validation Metrics
- Compare against GPS ground truth
- Analyze error statistics by terrain type
- Performance vs simulation baseline

## Implementation Checklist

- [ ] Download KAIST or EuRoC dataset
- [ ] Parse IMU data into STN format
- [ ] Download SRTM tiles for test area
- [ ] Implement GeoTIFF reader
- [ ] Download EGM2008 coefficients
- [ ] Implement gravity interpolation
- [ ] Build time synchronization
- [ ] Add coordinate transforms
- [ ] Create validation scripts
- [ ] Generate performance report

## Directory Structure
```
stn-v0.1/
├── data/
│   ├── real/
│   │   ├── kaist/         # Raw dataset
│   │   ├── terrain/       # DEM files
│   │   └── gravity/       # EGM2008 data
│   └── processed/         # Converted to STN format
├── cpp/
│   ├── real/
│   │   ├── data_parser.h/cpp
│   │   ├── terrain_real.h/cpp
│   │   └── gravity_real.h/cpp
│   └── utils/
│       └── coordinates.h/cpp
└── python/
    └── converters/
        ├── kaist_to_stn.py
        ├── dem_processor.py
        └── egm2008_extractor.py
```

## Expected Challenges

1. **Sensor Synchronization**: Real sensors have timing jitter
2. **Coordinate Systems**: Multiple reference frames to handle
3. **Data Gaps**: GPS outages, sensor dropouts
4. **Terrain Resolution**: Balancing accuracy vs memory
5. **Computational Load**: Real-time processing requirements

## Success Criteria

- Process 10 minutes of real flight data
- Achieve <50m CEP95 without GPS updates
- Maintain real-time performance (>10Hz)
- Handle GPS outages >60 seconds
- Demonstrate terrain correlation in mountainous areas

## Next Steps After Week 3

- Multi-sensor fusion (camera, lidar)
- Machine learning for terrain matching
- Adaptive filter tuning
- Field testing with live sensors