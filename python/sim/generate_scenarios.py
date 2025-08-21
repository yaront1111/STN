#!/usr/bin/env python3
"""Generate test scenarios: baseline, noisy radalt, aggressive turns"""

import pandas as pd
import numpy as np
from pathlib import Path

# Load baseline radalt data
baseline = pd.read_csv("data/radalt.csv")

# Scenario 1: Noisy radalt (+50% noise)
noisy = baseline.copy()
noise_scale = 1.5  # 50% more noise
np.random.seed(42)
noisy['agl_m'] += np.random.normal(0, 0.8 * (noise_scale - 1), len(noisy))
noisy.to_csv("data/radalt_noisy.csv", index=False)
print(f"Created noisy radalt scenario: std increased by {(noise_scale-1)*100:.0f}%")

# Scenario 2: Aggressive turns (need to modify IMU data)
imu_baseline = pd.read_csv("data/sim_imu.csv")
imu_aggressive = imu_baseline.copy()

# Add roll oscillations (banking turns)
t = imu_aggressive['t'].values
# Create realistic banking pattern: S-turns with varying frequency
bank_pattern = (
    0.3 * np.sin(2 * np.pi * 0.02 * t) +  # Slow turns
    0.2 * np.sin(2 * np.pi * 0.05 * t) +  # Medium turns
    0.1 * np.sin(2 * np.pi * 0.1 * t)     # Quick corrections
)

# Apply roll rate (gx) for banking
imu_aggressive['gx'] += bank_pattern * 0.5  # rad/s

# Add corresponding lateral acceleration (coordinated turn)
# In a coordinated turn: ay ≈ g * sin(φ) where φ is bank angle
# Integrate bank pattern to get approximate roll angle
roll_angle = np.cumsum(bank_pattern) * 0.01  # rough integration with dt=0.01
imu_aggressive['ay'] += 9.8 * np.sin(roll_angle) * 0.3  # scaled down

imu_aggressive.to_csv("data/sim_imu_aggressive.csv", index=False)
print(f"Created aggressive turns scenario: max roll rate ≈ {np.abs(bank_pattern).max()*0.5:.2f} rad/s")

# Scenario 3: Combined (noisy + aggressive)
combined = baseline.copy()
combined['agl_m'] += np.random.normal(0, 0.8 * (noise_scale - 1), len(combined))
combined.to_csv("data/radalt_combined.csv", index=False)
print(f"Created combined scenario: noisy radalt + aggressive IMU")

print("\nTest scenarios created:")
print("1. Baseline: data/sim_imu.csv + data/radalt.csv")
print("2. Noisy:    data/sim_imu.csv + data/radalt_noisy.csv")
print("3. Aggressive: data/sim_imu_aggressive.csv + data/radalt.csv")
print("4. Combined: data/sim_imu_aggressive.csv + data/radalt_combined.csv")