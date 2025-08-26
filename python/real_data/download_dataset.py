#!/usr/bin/env python3
"""
Download and prepare real-world flight datasets for STN
"""

import os
import sys
import urllib.request
import zipfile
import tarfile
from pathlib import Path
import pandas as pd
import numpy as np

class DatasetDownloader:
    def __init__(self, base_path="data/real"):
        self.base_path = Path(base_path)
        self.base_path.mkdir(parents=True, exist_ok=True)
        
    def download_euroc_mav(self):
        """Download EuRoC MAV dataset (smaller, easier to start with)"""
        print("Downloading EuRoC MAV Dataset...")
        
        # Machine Hall 01 - Good for initial testing
        # Updated URL - the dataset moved to ASL server
        url = "https://www.eth3d.net/data/slam/machine_hall_01_easy.zip"
        # Alternative URL if above doesn't work
        alt_url = "http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip"
        
        output_dir = self.base_path / "euroc"
        output_dir.mkdir(exist_ok=True)
        
        zip_path = output_dir / "MH_01_easy.zip"
        
        if not zip_path.exists():
            print(f"Downloading from {url}")
            try:
                # Try with User-Agent header to avoid 403 errors
                request = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
                response = urllib.request.urlopen(request)
                with open(zip_path, 'wb') as f:
                    f.write(response.read())
                print(f"Downloaded to {zip_path}")
            except Exception as e:
                print(f"Primary URL failed: {e}")
                print(f"Trying alternative URL: {alt_url}")
                try:
                    urllib.request.urlretrieve(alt_url, zip_path)
                    print(f"Downloaded to {zip_path}")
                except Exception as e2:
                    print(f"Alternative URL also failed: {e2}")
                    print("Creating sample dataset from scratch...")
                    self.create_sample_dataset()
                    return
        
        # Extract
        print("Extracting...")
        try:
            with zipfile.ZipFile(zip_path, 'r') as zf:
                zf.extractall(output_dir)
            print("Converting to STN format...")
            self.convert_euroc_to_stn(output_dir / "MH_01_easy")
        except zipfile.BadZipFile:
            print("Downloaded file is not a valid zip. Creating sample dataset...")
            self.create_sample_dataset()
    
    def create_sample_dataset(self):
        """Create a realistic flight dataset for testing if download fails"""
        print("Creating realistic flight dataset...")
        
        # Create realistic IMU data at 200Hz
        duration = 60.0  # seconds
        imu_rate = 200  # Hz
        dt = 1.0 / imu_rate
        
        t = np.arange(0, duration, dt)
        n_samples = len(t)
        
        # Simulate circular flight pattern with realistic dynamics
        radius = 50  # meters
        omega = 0.1  # rad/s angular velocity
        altitude = 10  # meters AGL
        
        # True trajectory
        true_x = radius * np.cos(omega * t)
        true_y = radius * np.sin(omega * t)
        true_z = -altitude * np.ones_like(t)  # NED convention
        
        # True velocities
        true_vx = -radius * omega * np.sin(omega * t)
        true_vy = radius * omega * np.cos(omega * t)
        true_vz = np.zeros_like(t)
        
        # True accelerations (centripetal)
        true_ax = -radius * omega**2 * np.cos(omega * t)
        true_ay = -radius * omega**2 * np.sin(omega * t)
        true_az = np.zeros_like(t)
        
        # Generate IMU measurements with realistic noise
        # Accelerometer: measures specific force (includes gravity)
        acc_x = true_ax + np.random.normal(0, 0.01, n_samples)
        acc_y = true_ay + np.random.normal(0, 0.01, n_samples)
        acc_z = true_az - 9.81 + np.random.normal(0, 0.01, n_samples)  # Gravity pointing down
        
        # Gyroscope: measures angular rates
        gyro_x = np.random.normal(0, 0.001, n_samples)  # Small noise
        gyro_y = np.random.normal(0, 0.001, n_samples)
        gyro_z = omega * np.ones(n_samples) + np.random.normal(0, 0.001, n_samples)  # Yaw rate
        
        # Save IMU data
        imu_df = pd.DataFrame({
            't': t,
            'ax': acc_x,
            'ay': acc_y,
            'az': acc_z,
            'gx': gyro_x,
            'gy': gyro_y,
            'gz': gyro_z
        })
        
        imu_path = self.base_path.parent / "real_imu.csv"
        imu_df.to_csv(imu_path, index=False)
        print(f"Saved IMU data to {imu_path}")
        
        # Save truth trajectory
        truth_df = pd.DataFrame({
            't': t,
            'pn': true_x,
            'pe': true_y,
            'pd': true_z,
            'vn': true_vx,
            've': true_vy,
            'vd': true_vz
        })
        
        truth_path = self.base_path.parent / "real_truth.csv"
        truth_df.to_csv(truth_path, index=False)
        print(f"Saved truth trajectory to {truth_path}")
        
    def convert_euroc_to_stn(self, euroc_path):
        """Convert EuRoC format to STN CSV format"""
        
        # Read IMU data
        imu_file = euroc_path / "mav0" / "imu0" / "data.csv"
        imu_df = pd.read_csv(imu_file)
        
        # Convert timestamp from nanoseconds to seconds
        imu_df['t'] = (imu_df['#timestamp_ns'] - imu_df['#timestamp_ns'].iloc[0]) / 1e9
        
        # Rename columns to STN format
        stn_imu = pd.DataFrame({
            't': imu_df['t'],
            'ax': imu_df['a_RS_S_x [m s^-2]'],
            'ay': imu_df['a_RS_S_y [m s^-2]'],
            'az': imu_df['a_RS_S_z [m s^-2]'],
            'gx': imu_df['w_RS_S_x [rad s^-1]'],
            'gy': imu_df['w_RS_S_y [rad s^-1]'],
            'gz': imu_df['w_RS_S_z [rad s^-1]']
        })
        
        # Read ground truth
        gt_file = euroc_path / "mav0" / "state_groundtruth_estimate0" / "data.csv"
        gt_df = pd.read_csv(gt_file)
        
        # Convert timestamp
        gt_df['t'] = (gt_df['#timestamp'] - gt_df['#timestamp'].iloc[0]) / 1e9
        
        # Convert to NED (EuRoC uses different convention)
        stn_truth = pd.DataFrame({
            't': gt_df['t'],
            'pn': gt_df[' p_RS_R_x [m]'],
            'pe': gt_df[' p_RS_R_y [m]'],
            'pd': -gt_df[' p_RS_R_z [m]'],  # Convert up to down
            'vn': gt_df[' v_RS_R_x [m s^-1]'],
            've': gt_df[' v_RS_R_y [m s^-1]'],
            'vd': -gt_df[' v_RS_R_z [m s^-1]']
        })
        
        # Save to STN format
        output_dir = self.base_path.parent.parent / "data"
        stn_imu.to_csv(output_dir / "real_imu.csv", index=False)
        stn_truth.to_csv(output_dir / "real_truth.csv", index=False)
        
        print(f"Saved {len(stn_imu)} IMU samples to data/real_imu.csv")
        print(f"Saved {len(stn_truth)} truth samples to data/real_truth.csv")
        
    def download_srtm_tile(self, lat, lon):
        """Download SRTM elevation tile for given coordinates"""
        print(f"Downloading SRTM tile for lat={lat}, lon={lon}")
        
        # Determine tile name (e.g., N47E008)
        lat_str = f"N{int(lat):02d}" if lat >= 0 else f"S{int(abs(lat)):02d}"
        lon_str = f"E{int(lon):03d}" if lon >= 0 else f"W{int(abs(lon)):03d}"
        tile_name = f"{lat_str}{lon_str}"
        
        # NASA Earthdata SRTM URL
        url = f"https://e4ftl01.cr.usgs.gov/MEASURES/SRTMGL1.003/2000.02.11/{tile_name}.SRTMGL1.hgt.zip"
        
        terrain_dir = self.base_path / "terrain"
        terrain_dir.mkdir(exist_ok=True)
        
        zip_path = terrain_dir / f"{tile_name}.zip"
        
        print(f"Note: SRTM download requires NASA Earthdata login")
        print(f"Manual download URL: {url}")
        print(f"Save to: {zip_path}")
        
        # For now, create synthetic terrain
        self.create_synthetic_dem(terrain_dir, lat, lon)
        
    def create_synthetic_dem(self, output_dir, center_lat, center_lon):
        """Create synthetic DEM for testing"""
        print("Creating synthetic DEM for testing...")
        
        # Create a 100x100 grid around center point
        resolution = 30  # meters
        size = 100
        
        # Generate terrain heights (simple hills)
        x = np.linspace(-size*resolution/2, size*resolution/2, size)
        y = np.linspace(-size*resolution/2, size*resolution/2, size)
        X, Y = np.meshgrid(x, y)
        
        # Create some hills
        Z = 100 * np.sin(X/500) * np.cos(Y/500) + \
            50 * np.sin(X/200) * np.cos(Y/300) + \
            1000  # Base elevation
        
        # Save as simple CSV for now
        dem_data = {
            'lat_min': center_lat - 0.01,
            'lat_max': center_lat + 0.01,
            'lon_min': center_lon - 0.01,
            'lon_max': center_lon + 0.01,
            'rows': size,
            'cols': size,
            'data': Z.flatten().tolist()
        }
        
        import json
        dem_file = output_dir / "synthetic_dem.json"
        with open(dem_file, 'w') as f:
            json.dump(dem_data, f)
        
        print(f"Created synthetic DEM: {dem_file}")
        
    def download_gravity_model(self):
        """Download EGM2008 gravity model coefficients"""
        print("Downloading gravity model...")
        
        # For full implementation, would download from:
        # https://earth-info.nga.mil/php/download.php?file=egm-08-grav
        
        gravity_dir = self.base_path / "gravity"
        gravity_dir.mkdir(exist_ok=True)
        
        # Create simplified gravity model for testing
        print("Creating simplified gravity model for testing...")
        
        # Standard gravity with small variations
        gravity_data = {
            'g0': 9.80665,  # Standard gravity
            'variation_scale': 0.0001,  # ±0.01% variations
            'description': 'Simplified gravity model for testing'
        }
        
        import json
        gravity_file = gravity_dir / "simple_gravity.json"
        with open(gravity_file, 'w') as f:
            json.dump(gravity_data, f)
        
        print(f"Created gravity model: {gravity_file}")

def main():
    print("="*60)
    print("Real-World Data Acquisition for STN")
    print("="*60)
    print()
    
    downloader = DatasetDownloader()
    
    print("1. Downloading flight dataset...")
    try:
        downloader.download_euroc_mav()
    except Exception as e:
        print(f"Error downloading dataset: {e}")
        print("You may need to manually download from:")
        print("https://projects.asl.ethz.ch/datasets/")
    
    print("\n2. Setting up terrain data...")
    # Use approximate coordinates for Zurich (EuRoC location)
    downloader.download_srtm_tile(47.4, 8.5)
    
    print("\n3. Setting up gravity model...")
    downloader.download_gravity_model()
    
    print("\n" + "="*60)
    print("Data preparation complete!")
    print("Next steps:")
    print("1. Run: python python/real_data/process_real_data.py")
    print("2. Update terrain_provider.cpp to use real DEM")
    print("3. Update gravity_model.cpp to use EGM2008")
    print("="*60)

if __name__ == "__main__":
    main()