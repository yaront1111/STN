#!/usr/bin/env python3
"""
Simplified training script for AI-Enhanced Navigation Models
Works with minimal dependencies (numpy, pandas)
"""

import os
import sys
import numpy as np
import pandas as pd
from pathlib import Path
import json

print("=" * 60)
print("AI-ENHANCED NAVIGATION TRAINING")
print("=" * 60)

# Load collected data
data_dir = Path("data")

# Check for IMU data
imu_files = list(data_dir.glob("imu/*.csv"))
print(f"\nFound {len(imu_files)} IMU data files")

# Check for gravity data
gravity_files = list(data_dir.glob("gravity/*.csv"))
print(f"Found {len(gravity_files)} gravity data files")

# Check for state data
state_files = list(data_dir.glob("states/*.csv"))
print(f"Found {len(state_files)} state data files")

# Load and analyze IMU data
if imu_files:
    print("\n" + "="*40)
    print("IMU DATA ANALYSIS")
    print("="*40)

    for f in imu_files[:2]:  # Analyze first 2 files
        df = pd.read_csv(f)
        print(f"\nFile: {f.name}")
        print(f"  Samples: {len(df)}")

        if len(df) > 0:
            # Calculate biases
            acc_bias = df[['bias_acc_x', 'bias_acc_y', 'bias_acc_z']].mean()
            gyro_bias = df[['bias_gyro_x', 'bias_gyro_y', 'bias_gyro_z']].mean()

            print(f"  Accelerometer bias (m/s²):")
            print(f"    X: {acc_bias['bias_acc_x']:.6f}")
            print(f"    Y: {acc_bias['bias_acc_y']:.6f}")
            print(f"    Z: {acc_bias['bias_acc_z']:.6f}")

            print(f"  Gyroscope bias (rad/s):")
            print(f"    X: {gyro_bias['bias_gyro_x']:.6f}")
            print(f"    Y: {gyro_bias['bias_gyro_y']:.6f}")
            print(f"    Z: {gyro_bias['bias_gyro_z']:.6f}")

            # Calculate errors
            acc_error = np.sqrt(
                ((df['acc_x'] - df['true_acc_x'])**2 +
                 (df['acc_y'] - df['true_acc_y'])**2 +
                 (df['acc_z'] - df['true_acc_z'])**2).mean()
            )
            print(f"  RMS accelerometer error: {acc_error:.6f} m/s²")

# Load and analyze gravity data
if gravity_files:
    print("\n" + "="*40)
    print("GRAVITY DATA ANALYSIS")
    print("="*40)

    for f in gravity_files[:2]:
        df = pd.read_csv(f)
        print(f"\nFile: {f.name}")
        print(f"  Samples: {len(df)}")

        if len(df) > 0:
            # Check if we have the expected columns
            if 'J2' in df.columns:
                print(f"  J2 invariant mean: {df['J2'].mean():.6f}")
                print(f"  J3 invariant mean: {df['J3'].mean():.6f}")
                print(f"  Laplacian mean: {df['laplacian'].mean():.6f}")

# Load and analyze state data
if state_files:
    print("\n" + "="*40)
    print("STATE DATA ANALYSIS")
    print("="*40)

    for f in state_files[:2]:
        df = pd.read_csv(f)
        print(f"\nFile: {f.name}")
        print(f"  Samples: {len(df)}")

        if len(df) > 0 and 'pos_error' in df.columns:
            print(f"  Position error:")
            print(f"    Mean: {df['pos_error'].mean():.2f} m")
            print(f"    Max: {df['pos_error'].max():.2f} m")
            print(f"    Final: {df['pos_error'].iloc[-1]:.2f} m")

            if 'vel_error' in df.columns:
                print(f"  Velocity error:")
                print(f"    Mean: {df['vel_error'].mean():.2f} m/s")
                print(f"    Max: {df['vel_error'].max():.2f} m/s")

# Generate training statistics
print("\n" + "="*40)
print("TRAINING DATA SUMMARY")
print("="*40)

total_imu_samples = sum(len(pd.read_csv(f)) for f in imu_files)
total_gravity_samples = sum(len(pd.read_csv(f)) for f in gravity_files) if gravity_files else 0
total_state_samples = sum(len(pd.read_csv(f)) for f in state_files) if state_files else 0

print(f"Total IMU samples: {total_imu_samples:,}")
print(f"Total gravity samples: {total_gravity_samples:,}")
print(f"Total state samples: {total_state_samples:,}")

# Check if we have enough data for training
min_samples_needed = 10000
if total_imu_samples >= min_samples_needed:
    print(f"\n✓ Sufficient IMU data for LSTM training")
else:
    print(f"\n✗ Need more IMU data ({min_samples_needed - total_imu_samples:,} more samples)")

if total_gravity_samples >= min_samples_needed:
    print(f"✓ Sufficient gravity data for U-Net training")
else:
    print(f"✗ Need more gravity data ({min_samples_needed - total_gravity_samples:,} more samples)")

# Save training metadata
metadata = {
    "imu_files": len(imu_files),
    "gravity_files": len(gravity_files),
    "state_files": len(state_files),
    "total_imu_samples": int(total_imu_samples),
    "total_gravity_samples": int(total_gravity_samples),
    "total_state_samples": int(total_state_samples),
    "ready_for_training": total_imu_samples >= min_samples_needed
}

with open("training_metadata.json", "w") as f:
    json.dump(metadata, f, indent=2)

print("\n" + "="*40)
print("NEXT STEPS")
print("="*40)

if metadata["ready_for_training"]:
    print("1. Data collection complete!")
    print("2. Install TensorFlow/PyTorch to train models")
    print("3. Run: python train_models.py --train-imu --train-gravity")
    print("4. Deploy ONNX models to C++")
else:
    print("1. Collect more data by running navigation system")
    print("2. Run: ./run_navigation_system --duration 600")
    print("3. Re-run this script to check data sufficiency")

print("\nTraining metadata saved to: training_metadata.json")
print("=" * 60)