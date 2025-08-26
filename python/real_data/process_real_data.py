#!/usr/bin/env python3
"""
Process real-world sensor data for STN navigation
"""

import pandas as pd
import numpy as np
from pathlib import Path
from scipy import interpolate
from scipy.spatial.transform import Rotation

class RealDataProcessor:
    def __init__(self):
        self.data_path = Path("data")
        
    def synchronize_sensors(self, imu_df, gps_df, baro_df=None):
        """Synchronize different sensor streams to common timeline"""
        
        # Find common time range
        t_min = max(imu_df['t'].min(), gps_df['t'].min())
        t_max = min(imu_df['t'].max(), gps_df['t'].max())
        
        # Create common timeline at IMU rate
        dt = np.median(np.diff(imu_df['t']))
        t_common = np.arange(t_min, t_max, dt)
        
        print(f"Synchronizing {len(t_common)} samples from {t_min:.2f} to {t_max:.2f} seconds")
        
        # Interpolate all sensors to common timeline
        synced_data = pd.DataFrame({'t': t_common})
        
        # IMU (no interpolation needed if already at target rate)
        for col in ['ax', 'ay', 'az', 'gx', 'gy', 'gz']:
            f = interpolate.interp1d(imu_df['t'], imu_df[col], 
                                    kind='linear', fill_value='extrapolate')
            synced_data[col] = f(t_common)
        
        # GPS (interpolate position, compute velocity)
        for col in ['lat', 'lon', 'alt']:
            if col in gps_df.columns:
                f = interpolate.interp1d(gps_df['t'], gps_df[col],
                                       kind='cubic', fill_value='extrapolate')
                synced_data[f'gps_{col}'] = f(t_common)
        
        # Barometer (if available)
        if baro_df is not None and 'pressure' in baro_df.columns:
            f = interpolate.interp1d(baro_df['t'], baro_df['pressure'],
                                   kind='linear', fill_value='extrapolate')
            synced_data['pressure'] = f(t_common)
            # Convert pressure to altitude
            synced_data['baro_alt'] = self.pressure_to_altitude(synced_data['pressure'])
        
        return synced_data
    
    def pressure_to_altitude(self, pressure_pa):
        """Convert pressure to altitude using standard atmosphere"""
        P0 = 101325  # Sea level pressure (Pa)
        L = 0.0065   # Temperature lapse rate (K/m)
        T0 = 288.15  # Sea level temperature (K)
        g = 9.80665  # Gravity (m/s²)
        R = 287.05   # Gas constant (J/kg·K)
        
        return T0/L * (1 - (pressure_pa/P0)**(R*L/g))
    
    def add_radar_altimeter(self, synced_df, terrain_provider):
        """Simulate radar altimeter using GPS position and terrain"""
        
        radalt_data = []
        
        for idx, row in synced_df.iterrows():
            if 'gps_lat' in row and 'gps_alt' in row:
                # Get terrain elevation at current position
                terrain_elev = terrain_provider.get_elevation(row['gps_lat'], row['gps_lon'])
                
                # AGL = GPS altitude - terrain elevation
                agl = row['gps_alt'] - terrain_elev
                
                # Add noise (radar altimeter typical accuracy ~1-2m)
                agl_noisy = agl + np.random.normal(0, 1.5)
                
                radalt_data.append({
                    't': row['t'],
                    'agl': agl_noisy,
                    'valid': agl > 10 and agl < 5000  # Valid range 10-5000m
                })
        
        return pd.DataFrame(radalt_data)
    
    def compute_truth_trajectory(self, truth_df, reference_point=None):
        """Process truth trajectory data (already in NED coordinates)"""
        
        # If truth data already has NED positions and velocities, return as-is
        if all(col in truth_df.columns for col in ['pn', 'pe', 'pd', 'vn', 've', 'vd']):
            return truth_df
        
        # If we have GPS data, convert to NED
        if 'gps_lat' in truth_df.columns:
            if reference_point is None:
                # Use first GPS point as reference
                reference_point = {
                    'lat': truth_df['gps_lat'].iloc[0],
                    'lon': truth_df['gps_lon'].iloc[0],
                    'alt': truth_df['gps_alt'].iloc[0]
                }
            
            # Convert to NED
            truth_data = []
            
            for idx, row in truth_df.iterrows():
                # Simple flat-earth approximation for small areas
                lat_to_m = 111320.0
                lon_to_m = 111320.0 * np.cos(reference_point['lat'] * np.pi / 180)
                
                pn = (row['gps_lat'] - reference_point['lat']) * lat_to_m
                pe = (row['gps_lon'] - reference_point['lon']) * lon_to_m
                pd = -(row['gps_alt'] - reference_point['alt'])
                
                truth_data.append({
                    't': row['t'],
                    'pn': pn,
                    'pe': pe,
                    'pd': pd
                })
            
            truth_df = pd.DataFrame(truth_data)
            
            # Compute velocities using finite differences
            dt = np.median(np.diff(truth_df['t']))
            truth_df['vn'] = np.gradient(truth_df['pn']) / dt
            truth_df['ve'] = np.gradient(truth_df['pe']) / dt
            truth_df['vd'] = np.gradient(truth_df['pd']) / dt
            
            # Smooth velocities
            from scipy.ndimage import uniform_filter1d
            for v in ['vn', 've', 'vd']:
                truth_df[v] = uniform_filter1d(truth_df[v], size=5, mode='nearest')
        
        return truth_df
    
    def validate_data_quality(self, imu_df, gps_df):
        """Check data quality and report issues"""
        
        issues = []
        
        # Check IMU data rate
        imu_dt = np.diff(imu_df['t'])
        imu_rate = 1.0 / np.median(imu_dt)
        if imu_rate < 50:
            issues.append(f"Low IMU rate: {imu_rate:.1f} Hz (recommend >100 Hz)")
        
        # Check for gaps
        max_gap = np.max(imu_dt)
        if max_gap > 0.1:
            issues.append(f"Large IMU gap detected: {max_gap:.3f} seconds")
        
        # Check GPS quality
        if 'hdop' in gps_df.columns:
            bad_hdop = (gps_df['hdop'] > 2.0).sum()
            if bad_hdop > len(gps_df) * 0.1:
                issues.append(f"Poor GPS quality: {bad_hdop/len(gps_df)*100:.1f}% with HDOP>2")
        
        # Check accelerometer bias
        static_periods = imu_df['ax'].rolling(100).std() < 0.1
        if static_periods.any():
            static_data = imu_df[static_periods]
            accel_bias = static_data[['ax', 'ay', 'az']].mean()
            expected_z = -9.81
            z_error = abs(accel_bias['az'] - expected_z)
            if z_error > 0.5:
                issues.append(f"Accelerometer Z bias: {z_error:.2f} m/s² from expected")
        
        if issues:
            print("⚠️  Data Quality Issues:")
            for issue in issues:
                print(f"  - {issue}")
        else:
            print("✅ Data quality checks passed")
        
        return len(issues) == 0
    
def main():
    print("="*60)
    print("Processing Real-World Data for STN")
    print("="*60)
    
    processor = RealDataProcessor()
    
    # Load real IMU data
    try:
        imu_df = pd.read_csv("data/real_imu.csv")
        truth_df = pd.read_csv("data/real_truth.csv")
        print(f"Loaded {len(imu_df)} IMU samples")
        print(f"Loaded {len(truth_df)} truth samples")
    except FileNotFoundError:
        print("❌ Real data not found. Run download_dataset.py first!")
        return
    
    # For EuRoC dataset, truth includes position
    # Convert to GPS-like format for processing
    gps_df = truth_df.copy()
    gps_df['gps_lat'] = 47.4 + gps_df['pn'] / 111320.0  # Approximate Zurich location
    gps_df['gps_lon'] = 8.5 + gps_df['pe'] / (111320.0 * np.cos(47.4 * np.pi / 180))
    gps_df['gps_alt'] = 400 - gps_df['pd']  # Approximate altitude
    
    # Synchronize sensors
    synced_df = processor.synchronize_sensors(imu_df, gps_df)
    
    # Validate data quality
    processor.validate_data_quality(imu_df, gps_df)
    
    # Save processed data
    synced_df.to_csv("data/real_synced.csv", index=False)
    print(f"Saved synchronized data to data/real_synced.csv")
    
    # Compute NED truth trajectory (use original truth_df which already has NED)
    truth_ned = processor.compute_truth_trajectory(truth_df)
    truth_ned.to_csv("data/real_truth_ned.csv", index=False)
    print(f"Saved NED truth to data/real_truth_ned.csv")
    
    print("\n" + "="*60)
    print("Real data processing complete!")
    print("Next steps:")
    print("1. Update main.cpp to use real_synced.csv")
    print("2. Run navigation with real data")
    print("3. Compare against real_truth_ned.csv")
    print("="*60)

if __name__ == "__main__":
    main()