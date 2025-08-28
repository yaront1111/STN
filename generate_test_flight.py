#!/usr/bin/env python3
"""
Generate realistic flight data for gravity navigation testing
Creates IMU data with realistic noise and dynamics
"""

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt

def generate_flight_trajectory(duration_s=600, dt=0.01):
    """Generate realistic aircraft flight trajectory"""
    
    t = np.arange(0, duration_s, dt)
    n_samples = len(t)
    
    # Initial position: Zurich airport (47.45°N, 8.55°E, 432m MSL)
    lat0 = 47.45 * np.pi / 180
    lon0 = 8.55 * np.pi / 180
    alt0 = 432.0
    
    # Flight profile: takeoff, climb, cruise, turn, descent
    phases = {
        'taxi': (0, 60),      # Taxi
        'takeoff': (60, 90),  # Takeoff roll
        'climb': (90, 300),   # Climb to altitude
        'cruise': (300, 450), # Cruise
        'turn': (450, 480),   # Banking turn
        'descent': (480, 600) # Descent
    }
    
    # Initialize state vectors
    lat = np.zeros(n_samples)
    lon = np.zeros(n_samples)
    alt = np.zeros(n_samples)
    vn = np.zeros(n_samples)  # North velocity
    ve = np.zeros(n_samples)  # East velocity
    vd = np.zeros(n_samples)  # Down velocity
    roll = np.zeros(n_samples)
    pitch = np.zeros(n_samples)
    yaw = np.zeros(n_samples)
    
    lat[0] = lat0
    lon[0] = lon0
    alt[0] = alt0
    
    for i in range(1, n_samples):
        current_t = t[i]
        
        # Determine flight phase
        if current_t < phases['taxi'][1]:
            # Taxi: slow ground movement
            vn[i] = 5.0 + np.random.randn() * 0.5
            ve[i] = 0.1 * np.sin(current_t * 0.1)
            vd[i] = 0
            pitch[i] = 0
            roll[i] = 0
            yaw[i] = 0
            
        elif current_t < phases['takeoff'][1]:
            # Takeoff: acceleration and rotation
            progress = (current_t - phases['takeoff'][0]) / 30
            vn[i] = 5.0 + 60.0 * progress  # Accelerate to 65 m/s
            ve[i] = 0
            vd[i] = -2.0 * progress if progress > 0.5 else 0  # Start climbing
            pitch[i] = 8.0 * progress * np.pi / 180  # Rotate to 8 degrees
            roll[i] = 0
            yaw[i] = 0
            
        elif current_t < phases['climb'][1]:
            # Climb: steady climb to cruise altitude
            vn[i] = 100.0 + np.random.randn() * 2.0  # ~200 knots
            ve[i] = 5.0 + np.random.randn() * 1.0
            vd[i] = -10.0 + np.random.randn() * 0.5  # Climb rate
            pitch[i] = (5.0 + np.random.randn() * 0.5) * np.pi / 180
            roll[i] = np.random.randn() * 2.0 * np.pi / 180
            yaw[i] = 0.1 * np.pi / 180
            
        elif current_t < phases['cruise'][1]:
            # Cruise: level flight
            vn[i] = 120.0 + np.random.randn() * 2.0  # ~240 knots
            ve[i] = 10.0 + np.random.randn() * 1.0
            vd[i] = np.random.randn() * 0.2  # Minor altitude variations
            pitch[i] = (2.0 + np.random.randn() * 0.5) * np.pi / 180
            roll[i] = np.random.randn() * 1.0 * np.pi / 180
            yaw[i] = 0.05 * np.pi / 180
            
        elif current_t < phases['turn'][1]:
            # Turn: banking turn
            progress = (current_t - phases['turn'][0]) / 30
            turn_rate = 3.0 * np.pi / 180  # 3 deg/s
            vn[i] = 120.0 * np.cos(turn_rate * current_t)
            ve[i] = 120.0 * np.sin(turn_rate * current_t)
            vd[i] = 0
            roll[i] = (30.0 * np.sin(np.pi * progress)) * np.pi / 180  # Bank angle
            pitch[i] = 2.0 * np.pi / 180
            yaw[i] = yaw[i-1] + turn_rate * dt
            
        else:
            # Descent
            progress = (current_t - phases['descent'][0]) / 120
            vn[i] = 100.0 - 20.0 * progress + np.random.randn() * 2.0
            ve[i] = 5.0 + np.random.randn() * 1.0
            vd[i] = 5.0 + 3.0 * progress  # Descent rate increases
            pitch[i] = (-2.0 - 3.0 * progress) * np.pi / 180
            roll[i] = np.random.randn() * 2.0 * np.pi / 180
            yaw[i] = yaw[i-1]
        
        # Update position (simplified flat Earth)
        R_earth = 6371000.0
        lat[i] = lat[i-1] + vn[i] * dt / R_earth
        lon[i] = lon[i-1] + ve[i] * dt / (R_earth * np.cos(lat[i]))
        alt[i] = alt[i-1] - vd[i] * dt
    
    return {
        't': t,
        'lat': lat,
        'lon': lon, 
        'alt': alt,
        'vn': vn,
        've': ve,
        'vd': vd,
        'roll': roll,
        'pitch': pitch,
        'yaw': yaw
    }

def generate_imu_measurements(trajectory, imu_rate=100):
    """Generate realistic IMU measurements from trajectory"""
    
    # Downsample to IMU rate
    traj_dt = trajectory['t'][1] - trajectory['t'][0]
    downsample = int(1.0 / (imu_rate * traj_dt))
    
    t_imu = trajectory['t'][::downsample]
    n_samples = len(t_imu)
    
    # IMU measurements
    acc_x = np.zeros(n_samples)
    acc_y = np.zeros(n_samples)
    acc_z = np.zeros(n_samples)
    gyro_x = np.zeros(n_samples)
    gyro_y = np.zeros(n_samples)
    gyro_z = np.zeros(n_samples)
    
    # IMU noise parameters (tactical grade)
    acc_noise_density = 100e-6 * 9.80665  # 100 μg/√Hz
    gyro_noise_density = 0.005 * np.pi / 180  # 0.005 deg/√Hz
    acc_bias_instability = 10e-6 * 9.80665  # 10 μg
    gyro_bias_instability = 0.5 * np.pi / 180 / 3600  # 0.5 deg/hr
    
    # Random walk biases
    acc_bias = np.random.randn(3) * acc_bias_instability
    gyro_bias = np.random.randn(3) * gyro_bias_instability
    
    dt = 1.0 / imu_rate
    
    for i in range(n_samples):
        idx = min(i * downsample, len(trajectory['t']) - 1)
        
        # Get attitude
        roll = trajectory['roll'][idx]
        pitch = trajectory['pitch'][idx]
        yaw = trajectory['yaw'][idx]
        
        # Create rotation matrix (body to nav)
        R = Rotation.from_euler('zyx', [yaw, pitch, roll]).as_matrix()
        
        # Compute accelerations (including gravity)
        if i > 0:
            # Numerical differentiation for accelerations
            dvn = (trajectory['vn'][idx] - trajectory['vn'][max(0, idx-downsample)]) / dt
            dve = (trajectory['ve'][idx] - trajectory['ve'][max(0, idx-downsample)]) / dt
            dvd = (trajectory['vd'][idx] - trajectory['vd'][max(0, idx-downsample)]) / dt
            
            # Navigation frame accelerations (including gravity)
            acc_nav = np.array([dvn, dve, dvd - 9.80665])  # Remove gravity
            
            # Transform to body frame
            acc_body = R.T @ acc_nav
        else:
            acc_body = np.array([0, 0, -9.80665])
        
        # Compute angular rates
        if i > 0:
            droll = (trajectory['roll'][idx] - trajectory['roll'][max(0, idx-downsample)]) / dt
            dpitch = (trajectory['pitch'][idx] - trajectory['pitch'][max(0, idx-downsample)]) / dt
            dyaw = (trajectory['yaw'][idx] - trajectory['yaw'][max(0, idx-downsample)]) / dt
            
            # Body angular rates (simplified)
            gyro_body = np.array([droll, dpitch, dyaw])
        else:
            gyro_body = np.array([0, 0, 0])
        
        # Add sensor errors
        acc_noise = np.random.randn(3) * acc_noise_density * np.sqrt(imu_rate)
        gyro_noise = np.random.randn(3) * gyro_noise_density * np.sqrt(imu_rate)
        
        # Update biases (random walk)
        acc_bias += np.random.randn(3) * acc_bias_instability * np.sqrt(dt)
        gyro_bias += np.random.randn(3) * gyro_bias_instability * np.sqrt(dt)
        
        # Final measurements
        acc_meas = acc_body + acc_bias + acc_noise
        gyro_meas = gyro_body + gyro_bias + gyro_noise
        
        acc_x[i] = acc_meas[0]
        acc_y[i] = acc_meas[1]
        acc_z[i] = acc_meas[2]
        gyro_x[i] = gyro_meas[0]
        gyro_y[i] = gyro_meas[1]
        gyro_z[i] = gyro_meas[2]
    
    # Get corresponding trajectory data
    lat_imu = trajectory['lat'][::downsample][:n_samples]
    lon_imu = trajectory['lon'][::downsample][:n_samples]
    alt_imu = trajectory['alt'][::downsample][:n_samples]
    
    return pd.DataFrame({
        't': t_imu,
        'acc_x': acc_x,
        'acc_y': acc_y,
        'acc_z': acc_z,
        'gyro_x': gyro_x,
        'gyro_y': gyro_y,
        'gyro_z': gyro_z,
        'lat_true': lat_imu * 180 / np.pi,
        'lon_true': lon_imu * 180 / np.pi,
        'alt_true': alt_imu,
        'temperature': 25.0 + np.random.randn(n_samples) * 0.5
    })

def add_gravity_measurements(df):
    """Add simulated gravity gradient and anomaly measurements"""
    
    n_samples = len(df)
    
    # Gravity gradient tensor (simulated)
    # Real gradiometer would measure at ~1 Hz with ~1 E noise
    gxx = 3.0 + 0.5 * np.sin(df['lat_true'].values * 10 * np.pi / 180)
    gyy = -1.5 + 0.3 * np.cos(df['lon_true'].values * 10 * np.pi / 180)
    gzz = -1.5 - gxx - gyy  # Laplace equation
    gxy = 0.1 * np.ones(n_samples)
    gxz = 0.2 * np.sin(df['t'].values * 0.01)
    gyz = 0.15 * np.cos(df['t'].values * 0.01)
    
    # Add measurement noise (1 Eötvös)
    gradient_noise = 1.0
    gxx += np.random.randn(n_samples) * gradient_noise
    gyy += np.random.randn(n_samples) * gradient_noise
    gzz += np.random.randn(n_samples) * gradient_noise
    gxy += np.random.randn(n_samples) * gradient_noise * 0.5
    gxz += np.random.randn(n_samples) * gradient_noise * 0.5
    gyz += np.random.randn(n_samples) * gradient_noise * 0.5
    
    # Gravity anomaly (from accelerometer)
    anomaly = 10.0 * np.sin(df['lat_true'].values * 20 * np.pi / 180)
    anomaly += 5.0 * np.cos(df['lon_true'].values * 20 * np.pi / 180)
    anomaly += np.random.randn(n_samples) * 0.5  # 0.5 mGal noise
    
    df['gradient_xx'] = gxx
    df['gradient_yy'] = gyy
    df['gradient_zz'] = gzz
    df['gradient_xy'] = gxy
    df['gradient_xz'] = gxz
    df['gradient_yz'] = gyz
    df['anomaly_mgal'] = anomaly
    
    return df

def plot_flight_data(df):
    """Plot the generated flight data"""
    
    fig, axes = plt.subplots(4, 2, figsize=(15, 12))
    
    # Trajectory
    axes[0, 0].plot(df['lon_true'], df['lat_true'])
    axes[0, 0].set_xlabel('Longitude (deg)')
    axes[0, 0].set_ylabel('Latitude (deg)')
    axes[0, 0].set_title('Flight Path')
    axes[0, 0].grid(True)
    
    # Altitude
    axes[0, 1].plot(df['t'], df['alt_true'])
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Altitude (m)')
    axes[0, 1].set_title('Altitude Profile')
    axes[0, 1].grid(True)
    
    # Accelerations
    axes[1, 0].plot(df['t'], df['acc_x'], label='X', alpha=0.7)
    axes[1, 0].plot(df['t'], df['acc_y'], label='Y', alpha=0.7)
    axes[1, 0].plot(df['t'], df['acc_z'], label='Z', alpha=0.7)
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Acceleration (m/s²)')
    axes[1, 0].set_title('IMU Accelerations')
    axes[1, 0].legend()
    axes[1, 0].grid(True)
    
    # Angular rates
    axes[1, 1].plot(df['t'], df['gyro_x'] * 180/np.pi, label='X', alpha=0.7)
    axes[1, 1].plot(df['t'], df['gyro_y'] * 180/np.pi, label='Y', alpha=0.7)
    axes[1, 1].plot(df['t'], df['gyro_z'] * 180/np.pi, label='Z', alpha=0.7)
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Angular Rate (deg/s)')
    axes[1, 1].set_title('IMU Gyroscope')
    axes[1, 1].legend()
    axes[1, 1].grid(True)
    
    # Gravity gradient diagonal
    axes[2, 0].plot(df['t'], df['gradient_xx'], label='Gxx', alpha=0.7)
    axes[2, 0].plot(df['t'], df['gradient_yy'], label='Gyy', alpha=0.7)
    axes[2, 0].plot(df['t'], df['gradient_zz'], label='Gzz', alpha=0.7)
    axes[2, 0].set_xlabel('Time (s)')
    axes[2, 0].set_ylabel('Gradient (Eötvös)')
    axes[2, 0].set_title('Gravity Gradient Tensor (Diagonal)')
    axes[2, 0].legend()
    axes[2, 0].grid(True)
    
    # Gravity gradient off-diagonal
    axes[2, 1].plot(df['t'], df['gradient_xy'], label='Gxy', alpha=0.7)
    axes[2, 1].plot(df['t'], df['gradient_xz'], label='Gxz', alpha=0.7)
    axes[2, 1].plot(df['t'], df['gradient_yz'], label='Gyz', alpha=0.7)
    axes[2, 1].set_xlabel('Time (s)')
    axes[2, 1].set_ylabel('Gradient (Eötvös)')
    axes[2, 1].set_title('Gravity Gradient Tensor (Off-diagonal)')
    axes[2, 1].legend()
    axes[2, 1].grid(True)
    
    # Gravity anomaly
    axes[3, 0].plot(df['t'], df['anomaly_mgal'])
    axes[3, 0].set_xlabel('Time (s)')
    axes[3, 0].set_ylabel('Anomaly (mGal)')
    axes[3, 0].set_title('Gravity Anomaly')
    axes[3, 0].grid(True)
    
    # Gradient trace (should be ~0)
    trace = df['gradient_xx'] + df['gradient_yy'] + df['gradient_zz']
    axes[3, 1].plot(df['t'], trace)
    axes[3, 1].axhline(y=0, color='r', linestyle='--', alpha=0.5)
    axes[3, 1].set_xlabel('Time (s)')
    axes[3, 1].set_ylabel('Trace (Eötvös)')
    axes[3, 1].set_title('Gradient Tensor Trace (Laplace Check)')
    axes[3, 1].grid(True)
    
    plt.tight_layout()
    plt.savefig('flight_data_visualization.png', dpi=150)
    print("Saved flight data visualization to flight_data_visualization.png")
    plt.close()

def main():
    print("Generating realistic flight data for gravity navigation testing...")
    print("=" * 60)
    
    # Generate trajectory
    print("1. Generating flight trajectory (10 minutes)...")
    trajectory = generate_flight_trajectory(duration_s=600, dt=0.01)
    
    # Generate IMU measurements
    print("2. Generating IMU measurements (100 Hz)...")
    df = generate_imu_measurements(trajectory, imu_rate=100)
    
    # Add gravity measurements
    print("3. Adding gravity gradient and anomaly measurements...")
    df = add_gravity_measurements(df)
    
    # Save to CSV
    output_file = 'flight_data.csv'
    df.to_csv(output_file, index=False)
    print(f"4. Saved data to {output_file}")
    
    # Print statistics
    print("\n" + "=" * 60)
    print("Flight Data Statistics:")
    print("=" * 60)
    print(f"Duration: {df['t'].max():.1f} seconds")
    print(f"Samples: {len(df)}")
    print(f"Sample rate: {len(df)/df['t'].max():.1f} Hz")
    print(f"Start position: {df['lat_true'].iloc[0]:.4f}°N, {df['lon_true'].iloc[0]:.4f}°E, {df['alt_true'].iloc[0]:.1f}m")
    print(f"End position: {df['lat_true'].iloc[-1]:.4f}°N, {df['lon_true'].iloc[-1]:.4f}°E, {df['alt_true'].iloc[-1]:.1f}m")
    print(f"Max altitude: {df['alt_true'].max():.1f}m")
    print(f"Distance traveled: ~{np.sum(np.sqrt(np.diff(df['lat_true'])**2 + np.diff(df['lon_true'])**2)) * 111000:.1f}m")
    
    print("\nIMU Statistics:")
    print(f"Acc magnitude: {np.sqrt((df[['acc_x', 'acc_y', 'acc_z']]**2).sum(axis=1)).mean():.2f} ± {np.sqrt((df[['acc_x', 'acc_y', 'acc_z']]**2).sum(axis=1)).std():.2f} m/s²")
    print(f"Gyro magnitude: {np.sqrt((df[['gyro_x', 'gyro_y', 'gyro_z']]**2).sum(axis=1)).mean()*180/np.pi:.2f} ± {np.sqrt((df[['gyro_x', 'gyro_y', 'gyro_z']]**2).sum(axis=1)).std()*180/np.pi:.2f} deg/s")
    
    print("\nGravity Measurements:")
    print(f"Gradient range: [{df[['gradient_xx', 'gradient_yy', 'gradient_zz']].min().min():.2f}, {df[['gradient_xx', 'gradient_yy', 'gradient_zz']].max().max():.2f}] Eötvös")
    print(f"Anomaly range: [{df['anomaly_mgal'].min():.2f}, {df['anomaly_mgal'].max():.2f}] mGal")
    trace = df['gradient_xx'] + df['gradient_yy'] + df['gradient_zz']
    print(f"Trace check (should be ~0): {trace.mean():.3f} ± {trace.std():.3f} Eötvös")
    
    # Plot data
    print("\n5. Generating visualization...")
    plot_flight_data(df)
    
    print("\n✅ Flight data generation complete!")
    print(f"Use '{output_file}' as input to the gravity navigator")

if __name__ == "__main__":
    main()