#!/usr/bin/env python3
"""
Advanced trajectory generator for STN testing
Implements complex flight patterns and realistic IMU error models
"""

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
from dataclasses import dataclass
from typing import Tuple, List
import matplotlib.pyplot as plt

@dataclass
class IMUConfig:
    """IMU error model parameters"""
    name: str
    gyro_bias_stability: float  # deg/hr
    gyro_arw: float             # deg/sqrt(hr)
    accel_bias_stability: float # mg
    accel_vrw: float            # m/s/sqrt(hr)
    gyro_scale_factor: float    # ppm
    accel_scale_factor: float   # ppm
    
    @classmethod
    def consumer(cls):
        """Consumer-grade MEMS IMU (smartphone quality)"""
        return cls(
            name="consumer",
            gyro_bias_stability=10.0,
            gyro_arw=0.3,
            accel_bias_stability=50.0,
            accel_vrw=0.1,
            gyro_scale_factor=1000,
            accel_scale_factor=1000
        )
    
    @classmethod
    def tactical(cls):
        """Tactical-grade MEMS IMU (drone quality)"""
        return cls(
            name="tactical",
            gyro_bias_stability=1.0,
            gyro_arw=0.1,
            accel_bias_stability=10.0,
            accel_vrw=0.03,
            gyro_scale_factor=100,
            accel_scale_factor=100
        )
    
    @classmethod
    def navigation(cls):
        """Navigation-grade IMU (aviation quality)"""
        return cls(
            name="navigation",
            gyro_bias_stability=0.01,
            gyro_arw=0.001,
            accel_bias_stability=0.1,
            accel_vrw=0.001,
            gyro_scale_factor=10,
            accel_scale_factor=10
        )


class TrajectoryGenerator:
    def __init__(self, dt: float = 0.005, g: float = 9.80665):
        """Initialize trajectory generator
        
        Args:
            dt: Time step (s) - 200Hz default
            g: Gravity constant (m/s²)
        """
        self.dt = dt
        self.g = g
        
    def generate_straight_level(self, duration: float, velocity: float, 
                               altitude: float) -> Tuple[pd.DataFrame, pd.DataFrame]:
        """Generate straight and level flight"""
        t = np.arange(0, duration, self.dt)
        n = len(t)
        
        # Truth trajectory
        truth = pd.DataFrame({
            't': t,
            'pn': velocity * t + 0.5,  # Start at 0.5m north
            'pe': np.zeros(n),
            'pd': -altitude * np.ones(n),  # Constant altitude (NED down)
            'vn': velocity * np.ones(n),
            've': np.zeros(n),
            'vd': np.zeros(n),
            'roll': np.zeros(n),
            'pitch': np.zeros(n),
            'yaw': np.zeros(n)
        })
        
        # Perfect IMU measurements (body = NED aligned)
        imu = pd.DataFrame({
            't': t,
            'ax': np.zeros(n),  # No acceleration in level flight
            'ay': np.zeros(n),
            'az': -self.g * np.ones(n),  # Upward specific force
            'gx': np.zeros(n),  # No rotation
            'gy': np.zeros(n),
            'gz': np.zeros(n)
        })
        
        return imu, truth
    
    def generate_coordinated_turn(self, duration: float, velocity: float,
                                  altitude: float, turn_rate: float) -> Tuple[pd.DataFrame, pd.DataFrame]:
        """Generate coordinated circular turn
        
        Args:
            turn_rate: Turn rate (deg/s)
        """
        t = np.arange(0, duration, self.dt)
        n = len(t)
        omega = np.radians(turn_rate)  # rad/s
        
        # Radius of turn
        radius = velocity / omega if omega != 0 else float('inf')
        
        # Bank angle for coordinated turn
        bank = np.arctan2(velocity * omega, self.g)
        
        # Truth trajectory (circular path)
        truth = pd.DataFrame({
            't': t,
            'pn': radius * np.sin(omega * t),
            'pe': radius * (1 - np.cos(omega * t)),
            'pd': -altitude * np.ones(n),
            'vn': velocity * np.cos(omega * t),
            've': velocity * np.sin(omega * t),
            'vd': np.zeros(n),
            'roll': bank * np.ones(n),
            'pitch': np.zeros(n),
            'yaw': omega * t
        })
        
        # IMU measurements (body frame)
        # Centripetal acceleration + gravity
        imu = pd.DataFrame({
            't': t,
            'ax': np.zeros(n),  # Forward velocity constant
            'ay': velocity * omega * np.ones(n),  # Centripetal
            'az': -self.g / np.cos(bank) * np.ones(n),  # Increased lift
            'gx': np.zeros(n),
            'gy': np.zeros(n),
            'gz': omega * np.ones(n)  # Yaw rate
        })
        
        return imu, truth
    
    def generate_climb_descend(self, duration: float, velocity: float,
                               start_alt: float, end_alt: float) -> Tuple[pd.DataFrame, pd.DataFrame]:
        """Generate climbing or descending flight"""
        t = np.arange(0, duration, self.dt)
        n = len(t)
        
        # Vertical speed for constant rate climb/descent
        climb_rate = (end_alt - start_alt) / duration
        
        # Flight path angle
        gamma = np.arctan2(-climb_rate, velocity)  # Negative because NED
        
        # Ground speed
        v_ground = np.sqrt(velocity**2 + climb_rate**2)
        
        # Truth trajectory
        truth = pd.DataFrame({
            't': t,
            'pn': velocity * t,
            'pe': np.zeros(n),
            'pd': -(start_alt + climb_rate * t),
            'vn': velocity * np.ones(n),
            've': np.zeros(n),
            'vd': -climb_rate * np.ones(n),
            'roll': np.zeros(n),
            'pitch': gamma * np.ones(n),
            'yaw': np.zeros(n)
        })
        
        # IMU measurements
        imu = pd.DataFrame({
            't': t,
            'ax': np.zeros(n),  # Constant speed
            'ay': np.zeros(n),
            'az': -self.g * np.cos(gamma) * np.ones(n),  # Reduced due to pitch
            'gx': np.zeros(n),
            'gy': np.zeros(n),  # Could add pitch rate at transitions
            'gz': np.zeros(n)
        })
        
        return imu, truth
    
    def generate_figure_eight(self, duration: float, velocity: float,
                             altitude: float, pattern_size: float = 500) -> Tuple[pd.DataFrame, pd.DataFrame]:
        """Generate figure-8 pattern (good for testing heading changes)"""
        t = np.arange(0, duration, self.dt)
        n = len(t)
        
        # Lemniscate parametric equations
        omega = 2 * np.pi / 30  # Complete pattern in 30s
        scale = pattern_size
        
        # Position
        pn = scale * np.sin(omega * t)
        pe = scale * np.sin(omega * t) * np.cos(omega * t)
        pd = -altitude * np.ones(n)
        
        # Velocity (derivatives)
        vn = scale * omega * np.cos(omega * t)
        ve = scale * omega * (np.cos(2*omega*t))
        vd = np.zeros(n)
        
        # Acceleration (second derivatives)
        an = -scale * omega**2 * np.sin(omega * t)
        ae = -scale * omega**2 * 2 * np.sin(2*omega*t)
        
        # Heading from velocity
        yaw = np.arctan2(ve, vn)
        
        # Bank angle for coordinated turn
        omega_turn = np.gradient(yaw) / self.dt
        bank = np.arctan2(velocity * omega_turn, self.g)
        
        truth = pd.DataFrame({
            't': t,
            'pn': pn,
            'pe': pe,
            'pd': pd,
            'vn': vn,
            've': ve,
            'vd': vd,
            'roll': bank,
            'pitch': np.zeros(n),
            'yaw': yaw
        })
        
        # Transform accelerations to body frame
        imu_data = []
        for i in range(n):
            # Rotation matrix from NED to body
            R = Rotation.from_euler('zyx', [yaw[i], 0, bank[i]])
            
            # Specific force in NED
            f_ned = np.array([an[i], ae[i], -self.g])
            
            # Transform to body
            f_body = R.inv().apply(f_ned)
            
            # Angular rates in body
            omega_body = R.inv().apply([0, 0, omega_turn[i] if i > 0 else 0])
            
            imu_data.append({
                't': t[i],
                'ax': f_body[0],
                'ay': f_body[1],
                'az': f_body[2],
                'gx': omega_body[0],
                'gy': omega_body[1],
                'gz': omega_body[2]
            })
        
        imu = pd.DataFrame(imu_data)
        
        return imu, truth
    
    def add_imu_errors(self, imu_perfect: pd.DataFrame, 
                      imu_config: IMUConfig) -> pd.DataFrame:
        """Add realistic IMU errors to perfect measurements"""
        
        imu = imu_perfect.copy()
        n = len(imu)
        dt = self.dt
        
        # Convert units
        gyro_bias_stability_rps = np.radians(imu_config.gyro_bias_stability / 3600)
        gyro_arw_rps = np.radians(imu_config.gyro_arw / 60)
        accel_bias_stability_mps2 = imu_config.accel_bias_stability * 0.00980665
        accel_vrw_mps = imu_config.accel_vrw / 60
        
        # Generate biases (random walk)
        gyro_bias = np.zeros((n, 3))
        accel_bias = np.zeros((n, 3))
        
        for i in range(1, n):
            # Gyro bias random walk
            gyro_bias[i] = gyro_bias[i-1] + np.random.normal(0, gyro_bias_stability_rps * np.sqrt(dt), 3)
            # Accel bias random walk
            accel_bias[i] = accel_bias[i-1] + np.random.normal(0, accel_bias_stability_mps2 * np.sqrt(dt), 3)
        
        # Add biases
        imu['gx'] += gyro_bias[:, 0]
        imu['gy'] += gyro_bias[:, 1]
        imu['gz'] += gyro_bias[:, 2]
        imu['ax'] += accel_bias[:, 0]
        imu['ay'] += accel_bias[:, 1]
        imu['az'] += accel_bias[:, 2]
        
        # Add white noise
        imu['gx'] += np.random.normal(0, gyro_arw_rps / np.sqrt(dt), n)
        imu['gy'] += np.random.normal(0, gyro_arw_rps / np.sqrt(dt), n)
        imu['gz'] += np.random.normal(0, gyro_arw_rps / np.sqrt(dt), n)
        imu['ax'] += np.random.normal(0, accel_vrw_mps / np.sqrt(dt), n)
        imu['ay'] += np.random.normal(0, accel_vrw_mps / np.sqrt(dt), n)
        imu['az'] += np.random.normal(0, accel_vrw_mps / np.sqrt(dt), n)
        
        # Add scale factor errors
        sf_gyro = 1 + imu_config.gyro_scale_factor * 1e-6
        sf_accel = 1 + imu_config.accel_scale_factor * 1e-6
        
        imu['gx'] *= sf_gyro
        imu['gy'] *= sf_gyro
        imu['gz'] *= sf_gyro
        imu['ax'] *= sf_accel
        imu['ay'] *= sf_accel
        imu['az'] *= sf_accel
        
        return imu


def test_trajectories():
    """Generate and visualize test trajectories"""
    gen = TrajectoryGenerator()
    
    # Generate different trajectories
    trajectories = {
        'straight': gen.generate_straight_level(60, 50, 1000),
        'turn': gen.generate_coordinated_turn(60, 50, 1000, 3),
        'climb': gen.generate_climb_descend(60, 50, 1000, 1500),
        'figure8': gen.generate_figure_eight(60, 50, 1000, 300)
    }
    
    # Plot trajectories
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    for (name, (imu, truth)), ax in zip(trajectories.items(), axes.flat):
        ax.plot(truth['pe'], truth['pn'])
        ax.set_xlabel('East (m)')
        ax.set_ylabel('North (m)')
        ax.set_title(f'{name.capitalize()} Trajectory')
        ax.grid(True)
        ax.axis('equal')
    
    plt.tight_layout()
    plt.savefig('data/trajectories.png')
    print("Saved trajectory plots to data/trajectories.png")
    
    # Test IMU error models
    imu_perfect = trajectories['turn'][0]
    
    for quality in ['consumer', 'tactical', 'navigation']:
        config = getattr(IMUConfig, quality)()
        imu_noisy = gen.add_imu_errors(imu_perfect, config)
        
        # Save to file
        filename = f'data/imu_{quality}.csv'
        imu_noisy.to_csv(filename, index=False)
        print(f"Saved {quality} IMU data to {filename}")


if __name__ == "__main__":
    test_trajectories()