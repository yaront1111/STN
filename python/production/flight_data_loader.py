#!/usr/bin/env python3
"""
Production flight data loader for real STN system
Loads real IMU and radar altimeter data from actual flights
"""

import pandas as pd
import numpy as np
from pathlib import Path

class FlightDataLoader:
    """Load and validate real flight data"""
    
    def __init__(self, data_dir="data/flight"):
        self.data_dir = Path(data_dir)
    
    def load_imu_data(self, filename):
        """
        Load real IMU data from CSV file
        Expected format: t, ax, ay, az, gx, gy, gz
        Units: accelerometer in m/s^2, gyro in rad/s
        """
        filepath = self.data_dir / filename
        if not filepath.exists():
            raise FileNotFoundError(f"IMU data file not found: {filepath}")
        
        imu = pd.read_csv(filepath)
        
        # Validate columns
        required_cols = ['t', 'ax', 'ay', 'az', 'gx', 'gy', 'gz']
        if not all(col in imu.columns for col in required_cols):
            raise ValueError(f"IMU data must have columns: {required_cols}")
        
        # Validate data ranges
        if imu['t'].diff().mean() > 0.1:
            print("Warning: IMU sample rate appears low (dt > 0.1s)")
        
        acc_magnitude = np.sqrt(imu['ax']**2 + imu['ay']**2 + imu['az']**2)
        if np.abs(acc_magnitude.mean() - 9.8) > 2.0:
            print("Warning: Accelerometer magnitude far from gravity")
        
        print(f"Loaded {len(imu)} IMU samples")
        print(f"  Time span: {imu['t'].min():.1f} to {imu['t'].max():.1f} seconds")
        print(f"  Sample rate: {1.0/imu['t'].diff().mean():.1f} Hz")
        
        return imu
    
    def load_radalt_data(self, filename):
        """
        Load real radar altimeter data
        Expected format: t, agl
        Units: meters above ground level
        """
        filepath = self.data_dir / filename
        if not filepath.exists():
            raise FileNotFoundError(f"Radar altimeter file not found: {filepath}")
        
        radalt = pd.read_csv(filepath)
        
        # Validate columns
        if 't' not in radalt.columns or 'agl' not in radalt.columns:
            raise ValueError("Radar altimeter data must have 't' and 'agl' columns")
        
        # Validate data
        if radalt['agl'].min() < 0:
            print("Warning: Negative AGL values detected")
        if radalt['agl'].max() > 10000:
            print("Warning: Very high AGL values (>10km)")
        
        print(f"Loaded {len(radalt)} radar altimeter samples")
        print(f"  AGL range: {radalt['agl'].min():.1f} to {radalt['agl'].max():.1f} meters")
        
        return radalt
    
    def load_truth_data(self, filename):
        """
        Load truth/reference data if available (e.g., from GPS/DGPS)
        Expected format: t, lat, lon, alt, vn, ve, vd
        """
        filepath = self.data_dir / filename
        if not filepath.exists():
            print(f"No truth data available at {filepath}")
            return None
        
        truth = pd.read_csv(filepath)
        print(f"Loaded {len(truth)} truth samples")
        return truth


if __name__ == "__main__":
    # Example usage
    loader = FlightDataLoader()
    
    print("Flight Data Loader - Production System")
    print("=" * 50)
    print("\nThis system works with real flight data only.")
    print("Expected data format:")
    print("  IMU: t, ax, ay, az, gx, gy, gz")
    print("  Radar: t, agl")
    print("\nPlace your flight data in: data/flight/")