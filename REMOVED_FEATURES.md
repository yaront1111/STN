# Removed Features Documentation

This file documents features that were removed during code cleanup on 2025-09-22.
These features can be re-integrated if needed in the future.

## 1. Particle Filter

**Purpose**: Multi-modal tracking with 500 particles for handling non-Gaussian distributions

**Removed from**: `run_navigation_system.cpp`
- Include: `#include "cpp/core/particle_filter.h"`
- Configuration (lines 852-861):
```cpp
ParticleFilter::Config pf_config;
pf_config.num_particles = 500;
pf_config.process_noise_std = Eigen::Vector3d(0.1, 0.1, 0.05);
pf_config.measurement_noise_std = 10.0;
pf_config.resample_threshold = 0.5;
pf_config.enable_adaptive_resampling = true;

ParticleFilter particle_filter(pf_config);
particle_filter.initialize(initial_state, initial_cov);
```
- Prediction (line 1002): `particle_filter.predict(acc, gyro, dt);`

**Reason for removal**: Competing with UKF, adding computational overhead without benefit

## 2. Adaptive Filter

**Purpose**: IMU bias estimation and correction

**Removed from**: `run_navigation_system.cpp`
- Include: `#include "cpp/core/adaptive_filter.h"`
- Configuration (lines 864-869):
```cpp
AdaptiveFilter::Config af_config;
af_config.window_size = 100;
af_config.adaptation_rate = 0.01;
af_config.enable_outlier_rejection = true;
AdaptiveFilter adaptive_filter(af_config);
```
- Updates (lines 950-960, already disabled):
```cpp
// adaptive_filter.updateBiasEstimate(acc, gyro);
// acc = adaptive_filter.correctAccelerometer(acc);
// gyro = adaptive_filter.correctGyroscope(gyro);
// adaptive_filter.updateScaleFactors(last_state);
```

**Reason for removal**: ML bias correction already handles this, redundant system

## 3. Terrain Matching

**Purpose**: Position fixes using radar altimeter and terrain database

**Removed from**: `run_navigation_system.cpp` (lines 1172-1222)
- Runs every 1 second
- Simulates 30m accuracy terrain matching
- Calls `ukf.updateGravityMapMatching()` - competing with gravity map matching!

**Key code**:
```cpp
if (config.enable_terrain && step % 100 == 0 && step > 0) {
    // Terrain matching with 30m noise
    std::normal_distribution<> terrain_match_noise(0, 30.0);
    Eigen::Vector3d terrain_pos = true_pos;
    terrain_pos(0) += terrain_match_noise(gen);
    terrain_pos(1) += terrain_match_noise(gen);

    Eigen::Matrix3d R_terrain = Eigen::Matrix3d::Identity();
    R_terrain(0,0) = 900.0;  // 30m std horizontal
    R_terrain(1,1) = 900.0;
    R_terrain(2,2) = 4.0;    // 2m std vertical

    ukf.updateGravityMapMatching(terrain_pos, R_terrain);
}
```

**Reason for removal**: Competing with gravity map matching, adding 30m noise every second

## 4. Terrain Provider (SRTM)

**Removed from**: `run_navigation_system.cpp` (lines 836-840)
```cpp
std::unique_ptr<SRTMProvider> terrain_provider;
if (config.enable_terrain) {
    logger.info("Initializing terrain provider...");
    terrain_provider = std::make_unique<SRTMProvider>();
}
```

## 5. Unused Map Matchers

**Files not integrated**:
- `cpp/core/gravity_map_matcher.cpp/h` - Exists but not used in main loop
- `cpp/core/hierarchical_map_matcher.cpp/h` - Exists but not used

**Note**: Current map matching is simulated with random noise, not using these actual matchers

## Re-integration Instructions

To re-add any feature:
1. Uncomment the relevant include
2. Re-add initialization code
3. Re-add update calls in main loop
4. Ensure no conflicts with existing systems
5. Test thoroughly for Grade requirements

## Performance Impact

Removing these features should:
- Reduce CPU usage by ~20% (no particle filter)
- Eliminate competing position updates
- Improve convergence by removing conflicting corrections
- Simplify debugging and tuning