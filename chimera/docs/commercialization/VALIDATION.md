# Validation & QA Strategy

**Track:** P0 (Critical for Quality)
**Owner:** QA Team
**Timeline:** Weeks 1-12 (continuous)

---

## Acceptance Test Matrix

### Core Metrics (Gate Criteria)

| Metric | Target | Measurement Method |
|--------|--------|-------------------|
| Altitude MAE (LiDAR) | ≤ 0.5m | Compare vs ground truth over full flight |
| Altitude MAE (baro-only) | ≤ 2.0m | Disable LiDAR, compare vs truth |
| Horizontal drift | ≤ 1 m/min (with OF) | Position error rate over cruise segment |
| Horizontal drift | ≤ 3 m/min (without OF) | Same, OF disabled |
| Lock time | ≤ 3s | Time from start to first reliable AGL |
| Recovery time | ≤ 2 keyframes | After baro takeover trigger |
| False takeover rate | < 1 per 10 hours | Count unnecessary takeovers |
| UKF latency P95 | < 0.5ms @ 400Hz | Measure propagate() duration |
| Smoother latency P95 | < 30ms @ 5Hz | Measure update() duration |
| CPU usage | < 25% (4-core i7) | Average over flight |
| Memory usage | < 300MB | Peak RSS |

### Dataset Pack (v1)

**Required Scenarios:**
1. **Standard takeoff/landing** (grass field, 60s)
   - Validates: Basic functionality, boot sequence, shutdown
2. **Low altitude cruise** (0.5-2m AGL, 120s)
   - Validates: LiDAR accuracy, stuck detection
3. **High altitude cruise** (10-50m AGL, 180s)
   - Validates: Baro accuracy, transition handling
4. **Low texture environment** (asphalt/water, 90s)
   - Validates: OF degradation handling, baro fallback
5. **Thermal variation** (-10°C to +60°C, bench test)
   - Validates: Thermal compensation, bias drift

**Data Format:**
- JSON (same as current flight_all_sensors_complete.json)
- Labeled ground truth (GNSS/mocap where available)
- Manifest with checksums (SHA256)
- Baseline report (expected metrics)

### Automated Test Harness

```bash
#!/bin/bash
# run_acceptance_tests.sh

for dataset in datasets/*.json; do
  echo "Testing $dataset..."
  
  # Run CHIMERA
  ./chimera_node_multi --data $dataset --flow ${dataset%.json}_flow.json \
    > logs/test_$(basename $dataset .json).log 2>&1
  
  # Score results
  python3 scripts/score_run.py \
    --telemetry logs/chimera_multi.jsonl \
    --ground_truth ${dataset%.json}_truth.csv \
    --output results/$(basename $dataset .json)_score.json
done

# Aggregate scores
python3 scripts/aggregate_scores.py results/*.json > acceptance_report.md
```

---

## CI/CD Pipeline

### Build Matrix

| OS | Compiler | Arch | GTSAM | Sanitizers |
|----|----------|------|-------|------------|
| Ubuntu 22.04 | GCC 11 | x86_64 | 4.2 | ASAN+UBSAN |
| Ubuntu 24.04 | GCC 13 | x86_64 | 4.2 | - |
| Ubuntu 22.04 | Clang 14 | x86_64 | 4.2 | TSAN |
| Ubuntu 22.04 | GCC 11 (cross) | aarch64 | 4.2 | - |

### Gates (PR checks)

1. **Build** - All platforms compile cleanly
2. **Unit tests** - All tests pass
3. **Static analysis** - clang-tidy clean
4. **Sanitizers** - No ASAN/UBSAN/TSAN violations
5. **Performance** - No regressions vs baseline (±5%)
6. **Coverage** - Critical paths > 80%

---

## HIL/Simulation Integration

### Gazebo Scenarios

```yaml
scenarios:
  - name: grass_field
    world: flat_grass.world
    sensors: [imu, lidar, baro, mag, camera]
    duration: 120s
    
  - name: asphalt_low_texture
    world: parking_lot.world
    sensors: [imu, lidar, baro, mag]  # No camera (low OF quality)
    duration: 90s
    
  - name: water_surface
    world: lake.world
    sensors: [imu, baro, mag]  # No lidar/camera
    duration: 60s
```

### Fault Injection

```python
# fault_injection_test.py
def test_dropped_of_frames():
    # Simulate camera occlusion
    for i in range(100, 200):  # Drop frames 100-200
        of_data[i] = None
    
    result = run_chimera(sensors={...})
    assert result.altitude_mae < 2.0  # Should fall back to baro
    assert "of_bad_streak" in result.telemetry

def test_lidar_plateau():
    # Simulate stuck LiDAR
    for i in range(50, 150):
        lidar_data[i].range_mean = 1.58  # Constant
    
    result = run_chimera(sensors={...})
    assert result.lidar_stuck_detected == True
    assert result.altitude_source == "baro"
```

---

## Long-Soak Testing

**Procedure:**
1. Run > 1 hour continuous operation
2. Monitor memory (Valgrind for leaks)
3. Track CPU/latency percentiles
4. Check for NaN/inf crashes
5. Validate contract compliance throughout

**Acceptance:**
- Zero crashes
- Memory stable (no leaks)
- CPU < budget for full duration
- Latency P95 within targets

---

**See Also:** [CI_CHECKLIST.yaml](../specs/CI_CHECKLIST.yaml)
