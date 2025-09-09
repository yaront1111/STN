# Grade A Navigation Requirements

## Performance Target
- **Position Accuracy**: < 50m RMS without GPS
- **Velocity Accuracy**: < 0.5 m/s RMS  
- **Attitude Accuracy**: < 0.5° RMS
- **Runtime**: Stable for > 60 seconds

## Current Status: Grade C
- Position Error: ~10,000m after 60s
- Velocity Error: ~150 m/s
- Attitude Error: ~5°

## Critical Issues Identified

### 1. Gravity Map Matching Failures
**Problem**: Correlation returns 0.0 for all search points
**Root Cause**: Synthetic gravity data lacks realistic spatial variations
**Solution Required**: 
- Real EGM2008 gravity anomaly data with actual spatial variations
- OR enhanced synthetic data with realistic anomaly patterns (±50 mGal variations)

### 2. Observability Limitations
**Problem**: Gravity gradients alone cannot constrain heading
**Current Mitigation**: Figure-8 and S-turn maneuvers
**Additional Required**:
- Magnetometer fusion (implemented but needs tuning)
- Star tracker integration for attitude reference
- Visual odometry for velocity constraints

### 3. Search Performance
**Issue**: Map matching searches 1257 points taking >100ms
**Optimization Needed**:
- Hierarchical search (coarse-to-fine)
- Parallel correlation computation
- Early termination on good matches (>0.95 correlation)

## Configuration Tuning Required

### UKF Parameters (cpp/core/ukf.cpp)
```cpp
// Current values need optimization
Q(0,0) = Q(1,1) = Q(2,2) = 1.0;     // Position noise - try 0.1
Q(3,3) = Q(4,4) = Q(5,5) = 0.1;     // Velocity noise - try 0.01  
Q(6,6) = Q(7,7) = Q(8,8) = 0.01;    // Attitude noise - try 0.001
Q(9,9) = Q(10,10) = Q(11,11) = 1e-6; // Acc bias - OK
Q(12,12) = Q(13,13) = Q(14,14) = 1e-8; // Gyro bias - OK
```

### Map Matcher Parameters (gravity_map_matcher.h:26-30)
```cpp
signature_length = 100;        // Increase from 50 for better correlation
correlation_threshold = 0.70;  // May need to lower to 0.60 initially
search_radius_m = 5000;        // Increase from 2000 for large errors
grid_resolution_m = 50;        // Decrease from 100 for finer search
min_measurements = 50;         // Increase from 30 for stability
```

### Sensor Fusion Weights
- Gravity anomaly: R = 100.0 (mGal)² - decrease to 10.0
- Gravity gradient: R = 1.0 (Eötvös)² - decrease to 0.1  
- Magnetometer: R = 0.1 (unit)² - OK
- Barometer: R = 100.0 (m)² - decrease to 10.0

## Test Improvements Needed

### 1. Realistic Gravity Data
```cpp
// Instead of: T_zz = -2.0 * G * M / (r*r*r);
// Need: Real EGM2008 anomalies with spatial variation
gravity_model.loadEGM2008("data/egm2008_to360.dat");
```

### 2. Enhanced Maneuvers
- Add vertical climbs/dives for altitude observability
- Increase turn rates for better heading observability
- Add acceleration/deceleration phases

### 3. Additional Sensors
```cpp
// Add these updates to test_grade_a.cpp
ukf.updateMagnetometer(mag_body, mag_ref, R_mag);
ukf.updateBarometer(baro_alt, baro_noise); 
ukf.updateTerrainAltitude(radar_alt, terrain_height, radar_noise);
```

## Path to Grade A

### Phase 1: Fix Map Matching (Priority 1)
1. Generate realistic gravity anomaly field
2. Verify correlation computation
3. Tune search parameters
4. Target: First successful match

### Phase 2: Sensor Fusion (Priority 2)  
1. Tune magnetometer weights
2. Add barometric updates
3. Implement ZUPT properly
4. Target: <1km error

### Phase 3: UKF Tuning (Priority 3)
1. Optimize process noise
2. Tune measurement noise
3. Add adaptive filtering
4. Target: <100m error

### Phase 4: Performance (Priority 4)
1. Optimize map search
2. Add parallel processing
3. Implement prediction
4. Target: <50m error (Grade A)

## Validation Metrics

### Grade A Criteria
- [ ] Position error < 50m RMS over 60s
- [ ] Velocity error < 0.5 m/s RMS
- [ ] Attitude error < 0.5° RMS  
- [ ] Map match success rate > 80%
- [ ] Computation time < 10ms per update
- [ ] Memory usage < 100MB
- [ ] No numerical instabilities (NaN/Inf)

### Current Performance
- [x] System runs without crashes
- [x] UKF numerically stable
- [ ] Map matching working
- [ ] Sensor fusion tuned
- [ ] Real data integration
- [ ] Performance targets met

## Next Steps

1. **Immediate**: Add realistic gravity anomaly generation
2. **Short-term**: Debug correlation computation  
3. **Medium-term**: Tune all parameters systematically
4. **Long-term**: Integrate with real hardware

## Commands for Testing

```bash
# Build
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build

# Run Grade A test
./build/test_grade_a

# Analyze results
python3 analyze_grade_a.py grade_a_results.csv
```