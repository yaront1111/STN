#!/usr/bin/env python3
"""
Generate realistic test flight data over real SRTM terrain
For testing and validation of the STN navigator
"""

import numpy as np
import pandas as pd
from pathlib import Path
import sys
import os

sys.path.append(os.path.dirname(__file__))
from srtm_reader import SRTMReader, ned_to_latlon

def generate_test_flight(
    duration_s=120,
    dt=0.01,
    velocity_mps=50.0,
    altitude_agl=1000.0,
    ref_lat=47.4,
    ref_lon=8.5,
    output_dir="data/flight"
):
    """
    Generate realistic flight data over real SRTM terrain
    
    Args:
        duration_s: Flight duration in seconds
        dt: IMU sample time (0.01 = 100Hz)
        velocity_mps: Cruise velocity (m/s)
        altitude_agl: Altitude above ground level (m)
        ref_lat: Starting latitude (degrees)
        ref_lon: Starting longitude (degrees)
    """
    
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Initialize SRTM reader
    print("Loading SRTM terrain data...")
    srtm = SRTMReader("data/terrain")
    
    # Get terrain elevation at start
    start_terrain = srtm.get_elevation(ref_lat, ref_lon)
    print(f"Starting terrain elevation: {start_terrain:.1f}m")
    
    # Generate time vector
    N = int(duration_s / dt)
    t = np.arange(N) * dt
    
    # Generate trajectory (straight and level flight north)
    # Position in NED frame
    pn = velocity_mps * t + 0.5  # Start at 0.5m north
    pe = np.zeros(N)  # Straight north (no east movement)
    
    # Get terrain elevation along path
    print("Computing terrain profile along flight path...")
    terrain_heights = []
    for i in range(0, N, 100):  # Sample every 100 points for speed
        lat, lon = ned_to_latlon(pn[i], pe[i], ref_lat, ref_lon)
        h = srtm.get_elevation(lat, lon)
        terrain_heights.append(h)
    
    # Interpolate terrain for all points
    terrain_indices = np.arange(0, N, 100)
    terrain_heights_full = np.interp(np.arange(N), terrain_indices, terrain_heights)
    
    # Set altitude to maintain constant AGL
    altitude_msl = terrain_heights_full + altitude_agl
    pdown = -altitude_msl  # Negative down in NED
    
    # Velocity (constant)
    vn = np.full(N, velocity_mps)
    ve = np.zeros(N)
    vd = np.zeros(N)  # Level flight
    
    # Generate IMU data
    print("Generating IMU measurements...")
    g = 9.80665
    
    # Initialize random generator for repeatability
    rng = np.random.default_rng(42)
    
    # Gyroscope (small noise around zero for straight flight)
    gyro_noise_std = 0.001  # rad/s (tactical grade)
    gyro = rng.normal(0, gyro_noise_std, (N, 3))
    
    # Accelerometer (specific force)
    # In level flight, accelerometer measures -g in z-axis
    acc = np.zeros((N, 3))
    acc[:, 2] = -g
    
    # Add accelerometer noise
    acc_noise_std = 0.02  # m/s^2 (tactical grade)
    acc += rng.normal(0, acc_noise_std, acc.shape)
    
    # Add small biases
    acc[:, 0] += 0.01  # Small x-bias
    acc[:, 1] += 0.005  # Small y-bias
    acc[:, 2] += 0.02  # Small z-bias
    gyro[:, 0] += 0.0001  # Small roll bias
    gyro[:, 1] += 0.0002  # Small pitch bias
    gyro[:, 2] += 0.0001  # Small yaw bias
    
    # Save IMU data
    imu = pd.DataFrame({
        't': t,
        'ax': acc[:, 0],
        'ay': acc[:, 1],
        'az': acc[:, 2],
        'gx': gyro[:, 0],
        'gy': gyro[:, 1],
        'gz': gyro[:, 2]
    })
    imu_file = output_dir / "imu.csv"
    imu.to_csv(imu_file, index=False)
    print(f"Saved IMU data: {imu_file}")
    
    # Generate radar altimeter measurements
    print("Generating radar altimeter measurements...")
    radalt_noise_std = 0.5  # meters
    agl_measurements = -pdown - terrain_heights_full + rng.normal(0, radalt_noise_std, N)
    
    radalt = pd.DataFrame({
        't': t,
        'agl': agl_measurements
    })
    radalt_file = output_dir / "radalt.csv"
    radalt.to_csv(radalt_file, index=False)
    print(f"Saved radar altimeter: {radalt_file}")
    
    # Save truth data for evaluation
    truth = pd.DataFrame({
        't': t,
        'pn': pn,
        'pe': pe,
        'pd': pdown,
        'vn': vn,
        've': ve,
        'vd': vd
    })
    truth_file = output_dir / "truth.csv"
    truth.to_csv(truth_file, index=False)
    print(f"Saved truth data: {truth_file}")
    
    # Print statistics
    print("\n" + "="*50)
    print("Flight Data Statistics:")
    print(f"  Duration: {duration_s}s")
    print(f"  Distance: {pn[-1]:.1f}m")
    print(f"  Altitude MSL: {altitude_msl[0]:.1f} to {altitude_msl[-1]:.1f}m")
    print(f"  AGL: {agl_measurements.mean():.1f} ± {agl_measurements.std():.1f}m")
    print(f"  Terrain variation: {terrain_heights_full.max() - terrain_heights_full.min():.1f}m")
    
    # Compute terrain slope statistics for TRN
    terrain_gradients = []
    for i in range(0, N, 100):
        lat, lon = ned_to_latlon(pn[i], pe[i], ref_lat, ref_lon)
        grad_n, grad_e = srtm.get_gradient(lat, lon)
        slope = np.sqrt(grad_n**2 + grad_e**2)
        terrain_gradients.append(slope)
    
    print(f"  Terrain slopes: {np.mean(terrain_gradients):.4f} ± {np.std(terrain_gradients):.4f}")
    print(f"  Max slope: {np.max(terrain_gradients):.4f}")
    print(f"  % slopes > 0.002: {100 * np.sum(np.array(terrain_gradients) > 0.002) / len(terrain_gradients):.1f}%")
    
    return imu_file, radalt_file, truth_file


if __name__ == "__main__":
    print("STN Test Flight Generator")
    print("="*50)
    
    # Generate test flight
    imu_file, radalt_file, truth_file = generate_test_flight()
    
    print("\nReady to test navigation:")
    print(f"  ./build/stn_navigator {radalt_file} {imu_file} data/flight/nav_output.csv")