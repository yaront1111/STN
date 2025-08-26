#!/usr/bin/env python3
"""
Universal flight data parser for various dataset formats
Supports EuRoC, KITTI, and custom CSV formats
"""

import pandas as pd
import numpy as np
from pathlib import Path
from abc import ABC, abstractmethod
from typing import Dict, Tuple, Optional
import yaml
import json

class DatasetParser(ABC):
    """Abstract base class for dataset parsers"""
    
    @abstractmethod
    def parse(self, data_path: str) -> Dict[str, pd.DataFrame]:
        """Parse dataset and return standardized dataframes"""
        pass
    
    def to_stn_format(self, data: Dict[str, pd.DataFrame]) -> Tuple[pd.DataFrame, pd.DataFrame]:
        """Convert to STN input format (IMU and truth)"""
        pass


class EuRoCParser(DatasetParser):
    """Parser for EuRoC MAV dataset"""
    
    def parse(self, data_path: str) -> Dict[str, pd.DataFrame]:
        """Parse EuRoC dataset structure
        
        Expected structure:
        data_path/
        ├── mav0/
        │   ├── imu0/
        │   │   └── data.csv
        │   ├── state_groundtruth_estimate0/
        │   │   └── data.csv
        │   └── ...
        """
        data_path = Path(data_path)
        result = {}
        
        # Parse IMU data
        imu_file = data_path / 'mav0' / 'imu0' / 'data.csv'
        if imu_file.exists():
            imu_df = pd.read_csv(imu_file)
            # EuRoC IMU columns: timestamp, wx, wy, wz, ax, ay, az
            imu_df['t'] = (imu_df['#timestamp'] - imu_df['#timestamp'].iloc[0]) * 1e-9
            result['imu'] = imu_df[['t', 'w_x', 'w_y', 'w_z', 'a_x', 'a_y', 'a_z']].copy()
            result['imu'].columns = ['t', 'gx', 'gy', 'gz', 'ax', 'ay', 'az']
        
        # Parse ground truth
        truth_file = data_path / 'mav0' / 'state_groundtruth_estimate0' / 'data.csv'
        if truth_file.exists():
            truth_df = pd.read_csv(truth_file)
            truth_df['t'] = (truth_df['#timestamp'] - truth_df['#timestamp'].iloc[0]) * 1e-9
            
            # Extract position, velocity, quaternion
            result['truth'] = pd.DataFrame({
                't': truth_df['t'],
                'px': truth_df['p_RS_R_x [m]'],
                'py': truth_df['p_RS_R_y [m]'],
                'pz': truth_df['p_RS_R_z [m]'],
                'vx': truth_df['v_RS_R_x [m s^-1]'],
                'vy': truth_df['v_RS_R_y [m s^-1]'],
                'vz': truth_df['v_RS_R_z [m s^-1]'],
                'qw': truth_df['q_RS_w []'],
                'qx': truth_df['q_RS_x []'],
                'qy': truth_df['q_RS_y []'],
                'qz': truth_df['q_RS_z []']
            })
        
        return result
    
    def to_stn_format(self, data: Dict[str, pd.DataFrame]) -> Tuple[pd.DataFrame, pd.DataFrame]:
        """Convert EuRoC data to STN format"""
        
        # IMU data is already in correct format
        imu_df = data['imu'].copy()
        
        # Convert truth to NED frame
        if 'truth' in data:
            truth = data['truth']
            
            # EuRoC uses different frame convention, convert to NED
            # This is simplified - actual conversion depends on specific dataset
            truth_ned = pd.DataFrame({
                't': truth['t'],
                'pn': truth['px'],  # Simplified mapping
                'pe': truth['py'],
                'pd': -truth['pz'],  # Down is negative Z
                'vn': truth['vx'],
                've': truth['vy'],
                'vd': -truth['vz']
            })
        else:
            truth_ned = None
        
        return imu_df, truth_ned


class KITTIParser(DatasetParser):
    """Parser for KITTI dataset"""
    
    def parse(self, data_path: str) -> Dict[str, pd.DataFrame]:
        """Parse KITTI dataset structure
        
        Expected structure:
        data_path/
        ├── oxts/
        │   └── data/
        │       ├── 0000000000.txt
        │       ├── 0000000001.txt
        │       └── ...
        └── oxts.txt (timestamps)
        """
        data_path = Path(data_path)
        result = {}
        
        # Load timestamps
        timestamp_file = data_path / 'oxts' / 'timestamps.txt'
        if timestamp_file.exists():
            timestamps = pd.read_csv(timestamp_file, header=None)
            timestamps = pd.to_datetime(timestamps[0])
            t = (timestamps - timestamps[0]).dt.total_seconds()
        else:
            # Generate timestamps if not available
            oxts_files = sorted((data_path / 'oxts' / 'data').glob('*.txt'))
            t = np.arange(len(oxts_files)) * 0.1  # Assume 10Hz
        
        # Parse OXTS data (GPS/INS)
        oxts_data = []
        oxts_dir = data_path / 'oxts' / 'data'
        
        for i, oxts_file in enumerate(sorted(oxts_dir.glob('*.txt'))):
            with open(oxts_file) as f:
                line = f.readline().strip().split()
                oxts_data.append([float(x) for x in line])
        
        if oxts_data:
            oxts_df = pd.DataFrame(oxts_data)
            
            # KITTI OXTS format:
            # 0-2: lat, lon, alt
            # 3-5: roll, pitch, yaw
            # 6-8: vn, ve, vf (velocities)
            # 9-11: vl, vu, vr (body velocities)
            # 12-14: ax, ay, az
            # 15-17: af, al, au (body accelerations)
            # 18-20: wx, wy, wz
            # 21-23: wf, wl, wu (body angular rates)
            # 24-28: GPS position accuracy/status
            
            result['imu'] = pd.DataFrame({
                't': t[:len(oxts_data)],
                'ax': oxts_df[12],
                'ay': oxts_df[13],
                'az': oxts_df[14],
                'gx': oxts_df[18],
                'gy': oxts_df[19],
                'gz': oxts_df[20]
            })
            
            result['gps'] = pd.DataFrame({
                't': t[:len(oxts_data)],
                'lat': oxts_df[0],
                'lon': oxts_df[1],
                'alt': oxts_df[2],
                'vn': oxts_df[6],
                've': oxts_df[7],
                'vd': -oxts_df[8]  # Convert to NED
            })
        
        return result
    
    def to_stn_format(self, data: Dict[str, pd.DataFrame]) -> Tuple[pd.DataFrame, pd.DataFrame]:
        """Convert KITTI data to STN format"""
        
        imu_df = data['imu'].copy()
        
        if 'gps' in data:
            gps = data['gps']
            
            # Convert GPS lat/lon to local NED
            lat0, lon0 = gps['lat'].iloc[0], gps['lon'].iloc[0]
            
            # Simple flat-earth approximation
            lat_to_m = 111320.0
            lon_to_m = 111320.0 * np.cos(np.radians(lat0))
            
            truth_ned = pd.DataFrame({
                't': gps['t'],
                'pn': (gps['lat'] - lat0) * lat_to_m,
                'pe': (gps['lon'] - lon0) * lon_to_m,
                'pd': -(gps['alt'] - gps['alt'].iloc[0]),
                'vn': gps['vn'],
                've': gps['ve'],
                'vd': gps['vd']
            })
        else:
            truth_ned = None
        
        return imu_df, truth_ned


class CustomCSVParser(DatasetParser):
    """Parser for custom CSV formats with configuration"""
    
    def __init__(self, config_file: Optional[str] = None):
        """Initialize with optional configuration file"""
        self.config = {}
        if config_file and Path(config_file).exists():
            with open(config_file) as f:
                if config_file.endswith('.yaml'):
                    self.config = yaml.safe_load(f)
                elif config_file.endswith('.json'):
                    self.config = json.load(f)
    
    def parse(self, data_path: str) -> Dict[str, pd.DataFrame]:
        """Parse custom CSV files based on configuration"""
        data_path = Path(data_path)
        result = {}
        
        # Default file patterns
        imu_pattern = self.config.get('imu_pattern', '*imu*.csv')
        truth_pattern = self.config.get('truth_pattern', '*truth*.csv')
        
        # Find and parse IMU files
        for imu_file in data_path.glob(imu_pattern):
            df = pd.read_csv(imu_file)
            
            # Map columns based on config or defaults
            col_map = self.config.get('imu_columns', {
                'timestamp': 't',
                'accel_x': 'ax',
                'accel_y': 'ay',
                'accel_z': 'az',
                'gyro_x': 'gx',
                'gyro_y': 'gy',
                'gyro_z': 'gz'
            })
            
            # Rename columns to standard names
            df_renamed = df.rename(columns={v: k for k, v in col_map.items() if v in df.columns})
            
            # Ensure time starts at zero
            if 't' in df_renamed.columns:
                df_renamed['t'] = df_renamed['t'] - df_renamed['t'].iloc[0]
            
            result['imu'] = df_renamed
            break  # Use first matching file
        
        # Find and parse truth files
        for truth_file in data_path.glob(truth_pattern):
            df = pd.read_csv(truth_file)
            
            # Map columns
            col_map = self.config.get('truth_columns', {
                'timestamp': 't',
                'pos_north': 'pn',
                'pos_east': 'pe',
                'pos_down': 'pd',
                'vel_north': 'vn',
                'vel_east': 've',
                'vel_down': 'vd'
            })
            
            df_renamed = df.rename(columns={v: k for k, v in col_map.items() if v in df.columns})
            
            if 't' in df_renamed.columns:
                df_renamed['t'] = df_renamed['t'] - df_renamed['t'].iloc[0]
            
            result['truth'] = df_renamed
            break
        
        return result
    
    def to_stn_format(self, data: Dict[str, pd.DataFrame]) -> Tuple[pd.DataFrame, pd.DataFrame]:
        """Convert custom data to STN format"""
        
        # Ensure required columns
        imu_df = data.get('imu', pd.DataFrame())
        if not imu_df.empty:
            required_cols = ['t', 'ax', 'ay', 'az', 'gx', 'gy', 'gz']
            for col in required_cols:
                if col not in imu_df.columns:
                    imu_df[col] = 0.0  # Fill missing with zeros
            imu_df = imu_df[required_cols]
        
        truth_df = data.get('truth', pd.DataFrame())
        if not truth_df.empty:
            required_cols = ['t', 'pn', 'pe', 'pd', 'vn', 've', 'vd']
            for col in required_cols:
                if col not in truth_df.columns:
                    truth_df[col] = 0.0
            truth_df = truth_df[required_cols]
        
        return imu_df, truth_df


class UniversalParser:
    """Universal parser that auto-detects dataset format"""
    
    @staticmethod
    def detect_format(data_path: str) -> str:
        """Detect dataset format from directory structure"""
        data_path = Path(data_path)
        
        # Check for EuRoC structure
        if (data_path / 'mav0' / 'imu0').exists():
            return 'euroc'
        
        # Check for KITTI structure
        if (data_path / 'oxts' / 'data').exists():
            return 'kitti'
        
        # Default to custom CSV
        return 'custom'
    
    @staticmethod
    def parse(data_path: str, format: Optional[str] = None) -> Tuple[pd.DataFrame, pd.DataFrame]:
        """Parse dataset with auto-detection or specified format"""
        
        if format is None:
            format = UniversalParser.detect_format(data_path)
        
        print(f"Detected dataset format: {format}")
        
        if format == 'euroc':
            parser = EuRoCParser()
        elif format == 'kitti':
            parser = KITTIParser()
        else:
            parser = CustomCSVParser()
        
        data = parser.parse(data_path)
        imu_df, truth_df = parser.to_stn_format(data)
        
        print(f"Parsed {len(imu_df)} IMU samples")
        if truth_df is not None:
            print(f"Parsed {len(truth_df)} truth samples")
        
        return imu_df, truth_df


def main():
    """Test parser with sample data"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Parse flight data for STN')
    parser.add_argument('data_path', help='Path to dataset')
    parser.add_argument('--format', choices=['euroc', 'kitti', 'custom'],
                       help='Dataset format (auto-detect if not specified)')
    parser.add_argument('--output', default='data/',
                       help='Output directory for processed data')
    args = parser.parse_args()
    
    # Parse data
    imu_df, truth_df = UniversalParser.parse(args.data_path, args.format)
    
    # Save to STN format
    output_dir = Path(args.output)
    output_dir.mkdir(exist_ok=True)
    
    imu_df.to_csv(output_dir / 'parsed_imu.csv', index=False)
    print(f"Saved IMU data to {output_dir / 'parsed_imu.csv'}")
    
    if truth_df is not None:
        truth_df.to_csv(output_dir / 'parsed_truth.csv', index=False)
        print(f"Saved truth data to {output_dir / 'parsed_truth.csv'}")


if __name__ == '__main__':
    main()