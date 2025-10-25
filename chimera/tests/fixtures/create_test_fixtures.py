#!/usr/bin/env python3
"""
Create test data fixtures from real flight data.
Extracts small subsets for fast unit testing.
"""

import json
import sys
from pathlib import Path

def load_flight_data(filepath):
    """Load the complete flight data JSON"""
    with open(filepath, 'r') as f:
        return json.load(f)

def extract_time_window(data, t_start, t_end, name="subset"):
    """Extract data within a time window"""
    subset = {}

    # Extract IMU data
    if 'imu' in data:
        subset['imu'] = [s for s in data['imu'] if t_start <= s['t'] <= t_end]
        print(f"  IMU: {len(subset['imu'])} samples")

    # Extract Magnetometer data
    if 'mag' in data:
        subset['mag'] = [s for s in data['mag'] if t_start <= s['t'] <= t_end]
        print(f"  Mag: {len(subset['mag'])} samples")

    # Extract LiDAR data
    if 'lidar' in data:
        subset['lidar'] = [s for s in data['lidar'] if t_start <= s['t'] <= t_end]
        print(f"  LiDAR: {len(subset['lidar'])} samples")

    # Extract Barometer data (if exists)
    if 'baro' in data:
        subset['baro'] = [s for s in data['baro'] if t_start <= s['t'] <= t_end]
        print(f"  Baro: {len(subset['baro'])} samples")

    # Extract Airspeed data (if exists)
    if 'airspeed' in data:
        subset['airspeed'] = [s for s in data['airspeed'] if t_start <= s['t'] <= t_end]
        print(f"  Airspeed: {len(subset['airspeed'])} samples")

    return subset

def create_boot_only_fixture(data, output_path):
    """Create fixture with only first 2s (for bootstrapping tests)"""
    print("\nCreating boot-only fixture (first 2s)...")

    if 'imu' not in data or len(data['imu']) == 0:
        print("ERROR: No IMU data found!")
        return False

    t0 = data['imu'][0]['t']
    t_end = t0 + 2.0

    subset = extract_time_window(data, t0, t_end, "boot")

    with open(output_path, 'w') as f:
        json.dump(subset, f, indent=2)

    print(f"✓ Saved to {output_path}")
    return True

def create_10s_fixture(data, output_path):
    """Create fixture with first 10s (for fast integration tests)"""
    print("\nCreating 10-second fixture...")

    if 'imu' not in data or len(data['imu']) == 0:
        print("ERROR: No IMU data found!")
        return False

    t0 = data['imu'][0]['t']
    t_end = t0 + 10.0

    subset = extract_time_window(data, t0, t_end, "10s")

    with open(output_path, 'w') as f:
        json.dump(subset, f, indent=2)

    print(f"✓ Saved to {output_path}")
    return True

def create_unit_test_data(output_path):
    """Create synthetic test data with known properties for unit tests"""
    print("\nCreating synthetic unit test data...")

    # Create minimal synthetic data with known values
    data = {
        'imu': [],
        'mag': [],
        'lidar': []
    }

    # 1 second of IMU data @ 400 Hz
    for i in range(400):
        t = i * 0.0025
        data['imu'].append({
            't': t,
            'gyro': [0.0, 0.0, 0.0],  # No rotation
            'accel': [0.0, 0.0, -9.81],  # Hovering (NED frame)
            'quat': [0.0, 0.0, 0.0, 1.0]  # Identity (xyzw)
        })

    # 20 mag samples @ 20 Hz (pointing north)
    for i in range(20):
        t = i * 0.05
        data['mag'].append({
            't': t,
            'mag': [0.3, 0.0, -0.4]  # Typical north-pointing NED mag field
        })

    # 10 LiDAR samples @ 10 Hz (10m altitude in mm)
    for i in range(10):
        t = i * 0.1
        data['lidar'].append({
            't': t,
            'range_min': 10000.0,  # 10m in millimeters
            'range_mean': 10050.0,
            'num_points': 100
        })

    with open(output_path, 'w') as f:
        json.dump(data, f, indent=2)

    print(f"  IMU: {len(data['imu'])} samples")
    print(f"  Mag: {len(data['mag'])} samples")
    print(f"  LiDAR: {len(data['lidar'])} samples")
    print(f"✓ Saved to {output_path}")
    return True

def main():
    # Paths
    data_dir = Path(__file__).parent.parent.parent / "DATA"
    fixtures_dir = Path(__file__).parent

    real_data_path = data_dir / "flight_all_sensors_complete.json"

    # Create fixtures directory if it doesn't exist
    fixtures_dir.mkdir(parents=True, exist_ok=True)

    print("CHIMERA Test Fixture Generator")
    print("=" * 50)

    # Check if real data exists
    if not real_data_path.exists():
        print(f"Warning: Real data file not found at {real_data_path}")
        print("Will only create synthetic data.")
        data = None
    else:
        print(f"Loading real data from: {real_data_path}")
        data = load_flight_data(real_data_path)
        print(f"✓ Loaded successfully")

        # Create real data fixtures
        create_boot_only_fixture(data, fixtures_dir / "data_boot_only.json")
        create_10s_fixture(data, fixtures_dir / "data_first_10s.json")

    # Create synthetic test data (always)
    create_unit_test_data(fixtures_dir / "data_synthetic_unit.json")

    print("\n" + "=" * 50)
    print("✓ All fixtures created successfully!")
    print(f"Location: {fixtures_dir}")
    print("\nFixtures created:")
    for f in fixtures_dir.glob("*.json"):
        size_kb = f.stat().st_size / 1024
        print(f"  - {f.name:30s} ({size_kb:8.1f} KB)")

if __name__ == "__main__":
    main()
