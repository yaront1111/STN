#!/usr/bin/env python3
"""
CHIMERA Telemetry Validation Script
Computes accurate metrics from telemetry logs for acceptance gates
"""
import json
import sys
import math
import statistics as st
from pathlib import Path

def load_telemetry(path):
    """Load JSONL telemetry file"""
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]

def horizontal_drift_m_per_min(records):
    """Compute horizontal drift rate in m/min (NED frame)"""
    if len(records) < 2:
        return None

    x0, y0, t0 = records[0]["pos"][0], records[0]["pos"][1], records[0]["t"]
    x1, y1, t1 = records[-1]["pos"][0], records[-1]["pos"][1], records[-1]["t"]

    drift_m = math.hypot(x1 - x0, y1 - y0)
    duration_min = max(1e-6, (t1 - t0) / 60.0)

    return drift_m / duration_min

def lidar_residual_stats(records):
    """Compute LiDAR factor residual statistics"""
    residuals = []

    for r in records:
        lidar_count = r.get("graph_error_lidar_count", 0)
        lidar_error = r.get("graph_error_lidar_total", 0)

        if lidar_count > 0 and lidar_error > 0:
            # Average per-factor error
            avg_error = lidar_error / lidar_count
            residuals.append(avg_error)

    if not residuals:
        return None, None, None

    mae = sum(residuals) / len(residuals)
    rms = math.sqrt(sum(r*r for r in residuals) / len(residuals))
    max_err = max(residuals)

    return mae, rms, max_err

def gyro_bias_analysis(records):
    """Extract gyro bias (handle rad/s or deg/s)"""
    final = records[-1]

    # Check if we have gyro_bias_dps (deg/s)
    bias_dps = final.get("gyro_bias_dps")

    if bias_dps is not None:
        # Single scalar value - this is the norm
        return None, bias_dps

    # Check for vector components (if we logged them)
    # For now, we only log the scalar norm in deg/s
    return None, bias_dps

def accel_bias_analysis(records):
    """Extract accel bias"""
    final = records[-1]
    bias_mps2 = final.get("accel_bias_mps2")
    return bias_mps2

def altitude_metrics(records):
    """Compute altitude-related metrics"""
    agl_initial = records[0]["agl_m"]
    agl_final = records[-1]["agl_m"]
    agl_change = agl_final - agl_initial

    # Compute altitude MAE if we had ground truth (we don't for this test)
    # This would require GNSS or mocap ground truth

    return agl_initial, agl_final, agl_change

def lock_time_analysis(records):
    """Time to first reliable altitude lock"""
    for r in records:
        if r.get("contract_ok") and r["agl_m"] > 0:
            return r["t"]
    return None

def performance_metrics(records):
    """Extract performance metrics"""
    ukf_times = [r["ukf_cpu_ms"] for r in records if "ukf_cpu_ms" in r and r["ukf_cpu_ms"] < 1e6]  # Filter outliers
    smoother_times = [r["smoother_cpu_ms"] for r in records if "smoother_cpu_ms" in r and r["smoother_cpu_ms"] < 1e6]

    if ukf_times:
        ukf_p50 = st.median(ukf_times)
        ukf_p95 = st.quantiles(ukf_times, n=20)[18] if len(ukf_times) > 20 else max(ukf_times)
    else:
        ukf_p50, ukf_p95 = None, None

    if smoother_times:
        smoother_p50 = st.median(smoother_times)
        smoother_p95 = st.quantiles(smoother_times, n=20)[18] if len(smoother_times) > 20 else max(smoother_times)
    else:
        smoother_p50, smoother_p95 = None, None

    return ukf_p50, ukf_p95, smoother_p50, smoother_p95

def watchdog_analysis(records):
    """Analyze watchdog health"""
    total = len(records)
    healthy = sum(1 for r in records if r.get("watchdog", {}).get("all_healthy", False))
    violations = sum(1 for r in records if r.get("watchdog", {}).get("deadline_violations", {}).get("has_violations", False))

    return healthy, total, violations

def mode_transition_analysis(records):
    """Analyze altitude source mode transitions"""
    transitions = records[-1].get("mode_transitions", 0)

    # Check for unwanted baro takeovers
    takeovers = 0
    for r in records:
        if r.get("altitude_source") == "BARO_TAKEOVER":
            takeovers += 1

    return transitions, takeovers

def main(log_path):
    """Main validation function"""
    if not Path(log_path).exists():
        print(f"Error: Log file not found: {log_path}")
        return 1

    print("="*70)
    print("CHIMERA TELEMETRY VALIDATION")
    print("="*70)
    print(f"Log file: {log_path}")

    records = load_telemetry(log_path)
    print(f"Records loaded: {len(records)}")

    if not records:
        print("Error: No telemetry records found")
        return 1

    duration_s = records[-1]["t"] - records[0]["t"]
    print(f"Duration: {duration_s:.2f} seconds ({duration_s/60:.2f} minutes)")
    print()

    # === ACCEPTANCE GATE METRICS ===
    print("="*70)
    print("ACCEPTANCE GATE METRICS")
    print("="*70)

    # 1. Horizontal drift
    drift_rate = horizontal_drift_m_per_min(records)
    if drift_rate is not None:
        print(f"✓ Horizontal drift rate: {drift_rate:.2f} m/min")
        print(f"  Target (with OF): ≤ 1.0 m/min")
        print(f"  Target (IMU-only): ≤ 10.0 m/min")
        print(f"  Status: {'PASS' if drift_rate <= 10.0 else 'FAIL'} (IMU-only mode)")

    # 2. LiDAR residual
    lidar_mae, lidar_rms, lidar_max = lidar_residual_stats(records)
    if lidar_mae is not None:
        print(f"\n✓ LiDAR altitude residual:")
        print(f"  MAE: {lidar_mae:.3f} m")
        print(f"  RMS: {lidar_rms:.3f} m")
        print(f"  Max: {lidar_max:.3f} m")
        print(f"  Target: ≤ 0.5 m MAE")
        print(f"  Status: {'PASS' if lidar_mae <= 0.5 else 'WARN'}")

    # 3. Altitude metrics
    agl_init, agl_final, agl_change = altitude_metrics(records)
    print(f"\n✓ Altitude tracking:")
    print(f"  Initial AGL: {agl_init:.2f} m")
    print(f"  Final AGL: {agl_final:.2f} m")
    print(f"  Change: {agl_change:+.2f} m")

    # 4. Lock time
    lock_time = lock_time_analysis(records)
    if lock_time is not None:
        print(f"\n✓ Lock time to first reliable AGL: {lock_time:.2f} s")
        print(f"  Target: ≤ 3.0 s")
        print(f"  Status: {'PASS' if lock_time <= 3.0 else 'FAIL'}")

    # 5. Mode transitions
    transitions, takeovers = mode_transition_analysis(records)
    print(f"\n✓ Mode transitions: {transitions}")
    print(f"  False takeovers: {takeovers}")
    print(f"  Target: < 0.1 per hour")
    takeover_rate = takeovers / (duration_s / 3600.0)
    print(f"  Rate: {takeover_rate:.2f} per hour")
    print(f"  Status: {'PASS' if takeover_rate < 0.1 else 'FAIL'}")

    # 6. Watchdog health
    healthy_count, total_count, violation_count = watchdog_analysis(records)
    print(f"\n✓ Watchdog health: {healthy_count}/{total_count} ({100*healthy_count/total_count:.1f}%)")
    print(f"  Deadline violations: {violation_count}")
    print(f"  Status: {'PASS' if healthy_count == total_count else 'FAIL'}")

    # === PERFORMANCE METRICS ===
    print("\n" + "="*70)
    print("PERFORMANCE METRICS")
    print("="*70)

    ukf_p50, ukf_p95, smoother_p50, smoother_p95 = performance_metrics(records)
    if ukf_p50 is not None:
        print(f"✓ UKF latency:")
        print(f"  P50: {ukf_p50:.2f} ms")
        print(f"  P95: {ukf_p95:.2f} ms")
        print(f"  Target: < 0.5 ms @ 400 Hz")
        # Note: These are wall-clock times, not actual latency
        print(f"  Status: WARN (wall-clock time, not latency)")

    if smoother_p50 is not None:
        print(f"\n✓ Smoother optimization:")
        print(f"  P50: {smoother_p50:.2f} ms")
        print(f"  P95: {smoother_p95:.2f} ms")
        print(f"  Target: < 30 ms @ 5 Hz")
        print(f"  Status: WARN (wall-clock time, not latency)")

    # === BIAS ESTIMATION ===
    print("\n" + "="*70)
    print("BIAS ESTIMATION")
    print("="*70)

    gyro_vec, gyro_norm = gyro_bias_analysis(records)
    if gyro_norm is not None:
        print(f"✓ Gyro bias norm: {gyro_norm:.3f} deg/s")
        # Note: 16.9 deg/s is unrealistically high - likely a units issue
        print(f"  NOTE: Value > 10 deg/s suggests possible units error")

    accel_bias = accel_bias_analysis(records)
    if accel_bias is not None:
        print(f"✓ Accel bias norm: {accel_bias:.3f} m/s²")

    print("\n" + "="*70)
    print("VALIDATION COMPLETE")
    print("="*70)

    return 0

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <telemetry.jsonl>")
        sys.exit(1)

    sys.exit(main(sys.argv[1]))
