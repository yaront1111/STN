# Tooling & Operations

**Track:** P0
**Owner:** DevOps Team
**Timeline:** Weeks 7-9

---

## Telemetry Schema v1

See [telemetry_schema_v1.json](../specs/telemetry_schema_v1.json) for full typed schema.

### Required Fields

```json
{
  "version": "1.0.0",
  "t": 12.34,
  "agl_m": 10.5,
  "alt_src": "lidar",
  "sensor_health": {
    "lidar": {"score": 1.0, "reason": ""},
    "baro": {"score": 0.8, "reason": "high_noise"},
    "mag": {"score": 1.0, "reason": ""},
    "of": {"score": 0.95, "reason": ""},
    "imu": {"score": 1.0, "reason": ""}
  },
  "contract_ok": true,
  "pos": [1.2, 3.4, -10.5],
  "vel": [5.0, 0.0, -0.5]
}
```

### Backward Compatibility

- Version field mandatory
- New fields added with defaults
- Never remove fields (deprecate with null)

---

## Crash Dumps & Repro Bundles

### Crash Dump Format

```json
{
  "timestamp": "2025-01-25T12:34:56Z",
  "version": "1.0.0",
  "exception": "NaN detected in velocity",
  "stack_trace": "...",
  "last_state": {
    "pose": {...},
    "velocity": {...},
    "covariance": [...]
  },
  "sensor_snapshot": {
    "last_100_imu": [...],
    "last_10_lidar": [...],
    ...
  }
}
```

### Repro Bundle

```bash
# Generate bundle
./chimera_node_multi --dump-on-crash crash_20250125.json

# Replay
./chimera_node_multi --replay crash_20250125.json
```

---

## Dashboards

### Grafana Export

```python
# Export telemetry to InfluxDB format
def export_to_influx(telemetry_jsonl):
    for line in open(telemetry_jsonl):
        data = json.loads(line)
        influx_line = f"chimera,host=uav1 agl={data['agl_m']},contract_ok={data['contract_ok']} {int(data['t']*1e9)}"
        print(influx_line)
```

### CLI Plotting

```bash
# Quick plot altitude
cat logs/chimera_multi.jsonl | jq -r '[.t, .agl_m] | @csv' | gnuplot -e "plot '-' with lines"
```

---

**See Also:** [telemetry_schema_v1.json](../specs/telemetry_schema_v1.json)
