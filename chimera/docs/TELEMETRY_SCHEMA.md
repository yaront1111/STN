# CHIMERA Telemetry Schema v1.0.0

## Overview

CHIMERA uses a **versioned JSON schema** for production telemetry output. This document describes the schema format, validation rules, and backward compatibility guarantees.

## Quick Start

### Validating Telemetry

```cpp
#include "utils/schema_validator.h"

using namespace chimera::utils;

// Validate telemetry JSON string
std::string json_line = "{\"t\":1.5,\"pos\":[0,0,-2.5],...}";
auto result = SchemaValidator::validate(json_line);

if (result.valid) {
  std::cout << "✓ Telemetry valid\n";
} else {
  std::cerr << result.detailedReport() << "\n";
}
```

### Checking Version Compatibility

```cpp
// Check if old logs can be read
bool compat = SchemaValidator::isVersionCompatible("1.0.0");

// Check backward compatibility
bool backward_compat = CompatibilityChecker::isBackwardCompatible(
  "1.0.0",  // old version
  "1.1.0"   // new version
);
```

---

## Schema Format

### File Location

- **Schema definition:** `schema/telemetry_v1.json`
- **Validator:** `src/utils/schema_validator.h` and `.cpp`
- **Tests:** `tests/cpp/test_schema_validation.cpp`

### Schema Version

Current version: **1.0.0**

Format: `MAJOR.MINOR.PATCH`

- **MAJOR:** Breaking changes (incompatible schema)
- **MINOR:** Backward-compatible additions (new optional fields)
- **PATCH:** Documentation or clarifications only

---

## Required Fields

### Top-Level

All telemetry messages **must** include:

| Field | Type | Description |
|-------|------|-------------|
| `t` | number | Timestamp (seconds, >= 0) |
| `pos` | array[3] | Position [x, y, z] in NED (meters) |
| `vel` | array[3] | Velocity [vx, vy, vz] in NED (m/s) |
| `agl_m` | number | Altitude above ground (meters, >= 0) |
| `contract_ok` | boolean | Contract monitor status |
| `altitude_source` | enum | Current altitude source (NONE, LIDAR, BARO, BARO_TAKEOVER) |
| `sensor_health` | object | Per-sensor health monitoring (nested) |
| `watchdog` | object | Watchdog system status (nested) |

### Sensor Health (Nested Object)

Must include all 5 sensors:

**`sensor_health.lidar`:**
- `score` (number, 0-1): Health score
- `is_stuck` (boolean): Stuck detection (dual-gate)
- `noise_level` (number, >= 0): Measurement noise (meters)
- `reason` (string): Health degradation reason

**`sensor_health.baro`:**
- `score` (number, 0-1)
- `noise_level` (number, >= 0): Noise (meters)
- `drift_rate` (number): Drift rate (m/s)
- `rapid_change` (boolean): Rapid pressure change detected
- `reason` (string)

**`sensor_health.mag`:**
- `score` (number, 0-1)
- `norm` (number, >= 0): Magnetic field norm
- `interference` (boolean): Magnetic interference detected
- `reason` (string)

**`sensor_health.of`:**
- `score` (number, 0-1)
- `texture_quality` (number, >= 0): Image texture quality
- `saturated` (boolean): Image saturation detected
- `reason` (string)

**`sensor_health.imu`:**
- `score` (number, 0-1)
- `temperature` (number): IMU temperature (Celsius)
- `gyro_saturated` (boolean): Gyro saturation
- `accel_saturated` (boolean): Accelerometer saturation
- `reason` (string)

### Watchdog (Nested Object)

**`watchdog`:**
- `all_healthy` (boolean): All watchdog checks passed
- `failure_count` (integer, >= 0): Total failures
- `deadline_violations` (object):
  - `has_violations` (boolean)
  - `timed_out_sensors` (array of strings): ["imu", "lidar", ...]

---

## Optional Fields

The following fields are optional but recommended for full functionality:

| Field | Type | Description |
|-------|------|-------------|
| `z_eff_m` | number | Effective depth for optical flow |
| `mag_updates` | integer | Total magnetometer updates |
| `lidar_updates` | integer | Total LiDAR updates |
| `baro_updates` | integer | Total barometer updates |
| `of_updates` | integer | Total optical flow updates |
| `of_rays_used` | integer | Optical flow rays used |
| `of_rays_rejected` | integer | Optical flow rays rejected |
| `of_rms_px` | number | Optical flow RMS residual (pixels) |
| `of_health` | enum | "healthy", "degraded", "bad" |
| `wind` | array[2] | Wind velocity [wx, wy] (m/s) |
| `gyro_bias_dps` | number | Gyro bias magnitude (deg/s) |
| `accel_bias_mps2` | number | Accel bias magnitude (m/s^2) |
| `graph_error_*` | number | Factor graph error metrics |
| `ukf_cpu_ms` | number | UKF CPU time (ms) |
| `smoother_cpu_ms` | number | Smoother CPU time (ms) |
| `anchor` | boolean | Altitude reanchor applied |
| `mode_transitions` | integer | Total mode transitions |

---

## Validation Rules

### Type Validation

- **Numbers:** Must be valid JSON numbers (no NaN, Inf)
- **Booleans:** Must be `true` or `false` (lowercase)
- **Strings:** Must be double-quoted JSON strings
- **Arrays:** Must have correct length (e.g., `pos` must have 3 elements)

### Range Validation

- `t >= 0` (timestamp cannot be negative)
- `agl_m >= 0` (altitude cannot be negative)
- Health scores: `0.0 <= score <= 1.0`
- Counts: `>= 0` (update counts, failure counts)

### Enum Validation

- `altitude_source`: Must be one of `["NONE", "LIDAR", "BARO", "BARO_TAKEOVER"]`
- `of_health`: Must be one of `["healthy", "degraded", "bad"]`
- `timed_out_sensors`: Elements must be one of `["imu", "lidar", "baro", "mag", "optical_flow"]`

---

## Backward Compatibility

### Compatibility Rules

1. **Same MAJOR version** → Compatible (can read old logs)
2. **Different MAJOR version** → Incompatible (breaking changes)
3. **MINOR version increase** → Backward compatible (new optional fields added)
4. **MINOR version decrease** → Incompatible (cannot read newer logs)
5. **PATCH version** → Always compatible (documentation only)

### Examples

```
1.0.0 → 1.0.1  ✓ Compatible (patch)
1.0.0 → 1.1.0  ✓ Backward compatible (new optional fields)
1.1.0 → 1.0.0  ✗ Incompatible (cannot read newer logs)
1.5.0 → 2.0.0  ✗ Breaking change (incompatible)
```

### Deprecation Policy

When deprecating fields:

1. **v1.x.0:** Mark field as deprecated (add new field)
2. **v1.y.0:** Both old and new fields supported (transition period)
3. **v2.0.0:** Old field removed (breaking change)

**Example:** `alt_src` (deprecated) → `altitude_source` (new)

---

## Usage Examples

### Validating Real Telemetry

```cpp
#include "utils/telemetry.h"
#include "utils/schema_validator.h"

Telemetry telem("logs/output.jsonl");

// Generate telemetry
std::string json = generateTelemetryJSON();

// Validate before writing
auto result = SchemaValidator::validate(json);
if (!result.valid) {
  std::cerr << "Invalid telemetry: " << result.summary() << "\n";
  // Handle error (log, fix, skip)
}

telem.writeLine(json);
```

### Batch Validation

```cpp
std::ifstream file("logs/chimera_multi.jsonl");
std::string line;
int valid_count = 0;
int invalid_count = 0;

while (std::getline(file, line)) {
  auto result = SchemaValidator::validate(line);
  if (result.valid) {
    valid_count++;
  } else {
    invalid_count++;
    std::cerr << "Line " << (valid_count + invalid_count)
              << ": " << result.summary() << "\n";
  }
}

std::cout << "Validated " << valid_count << " / "
          << (valid_count + invalid_count) << " lines\n";
```

### Version Migration

```cpp
// Check if migration needed
if (!SchemaValidator::isVersionCompatible("0.9.0")) {
  std::string guide = CompatibilityChecker::getMigrationGuide(
    "0.9.0",  // current log version
    SchemaValidator::getSchemaVersion()  // target version
  );
  std::cout << guide << "\n";
}
```

---

## Production Deployment

### Pre-Flight Checks

1. **Schema Version:** Verify schema version matches expected
   ```cpp
   assert(SchemaValidator::getSchemaVersion() == "1.0.0");
   ```

2. **Sample Validation:** Validate first 10 telemetry messages
   ```cpp
   for (int i = 0; i < 10; i++) {
     auto result = SchemaValidator::validate(sample[i]);
     assert(result.valid);
   }
   ```

3. **Log Rotation:** Include schema version in log filenames
   ```
   chimera_multi_v1.0.0_2025-10-25.jsonl
   ```

### CI/CD Integration

Add schema validation to acceptance tests:

```bash
# Extract telemetry from test run
./build/chimera_node_multi --data test.json > /tmp/telem.jsonl

# Validate schema compliance
python scripts/validate_telemetry_schema.py /tmp/telem.jsonl
```

---

## Troubleshooting

### Common Validation Errors

**Error:** `Missing required field: sensor_health`
- **Cause:** Telemetry missing sensor health object
- **Fix:** Ensure `emitTelemetry()` includes all required fields

**Error:** `Field 'agl_m' must be a number >= 0`
- **Cause:** Negative altitude or non-numeric value
- **Fix:** Check altitude computation, ensure no NaN values

**Error:** `Missing sensor in sensor_health: lidar`
- **Cause:** LiDAR sensor health not included
- **Fix:** Add all 5 sensors (lidar, baro, mag, of, imu) to sensor_health

**Warning:** `Using deprecated field 'alt_src' without 'altitude_source'`
- **Cause:** Using old field name
- **Fix:** Replace `alt_src` with `altitude_source`

---

## Future Enhancements

### v1.1.0 (Planned)

**Additions (backward compatible):**
- `thermal_state`: Thermal model state (optional)
- `cpu_usage_percent`: System CPU usage (optional)
- `memory_usage_mb`: Memory footprint (optional)

### v2.0.0 (Future)

**Breaking changes:**
- Remove deprecated `alt_src` field
- Change `pos`/`vel` to nested objects with explicit `x`, `y`, `z` fields
- Add required `schema_version` field to every message

---

## References

- **JSON Schema Draft 7:** http://json-schema.org/draft-07/schema
- **CHIMERA Telemetry Schema:** `schema/telemetry_v1.json`
- **Implementation:** `src/utils/schema_validator.h`
- **Tests:** `tests/cpp/test_schema_validation.cpp` (21 tests)

---

**Last Updated:** October 25, 2025
**Schema Version:** 1.0.0
**Status:** Production-ready ✅
