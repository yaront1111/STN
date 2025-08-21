#!/usr/bin/env python3
"""Run STN trajectory simulation with configurable parameters."""

import argparse
from ins_sim import simulate_straight_and_level

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='STN Trajectory Simulator')
    parser.add_argument('--duration', type=float, default=120, help='Simulation duration (s)')
    parser.add_argument('--velocity', type=float, default=50, help='Vehicle velocity (m/s)')
    parser.add_argument('--altitude', type=float, default=1000, help='Initial altitude (m)')
    parser.add_argument('--trajectory', type=str, default='straight', 
                       choices=['straight', 'turn', 'climb', 'descend', 'orbit'],
                       help='Trajectory type')
    parser.add_argument('--imu-quality', type=str, default='tactical',
                       choices=['consumer', 'tactical', 'navigation', 'perfect'],
                       help='IMU quality level')
    args = parser.parse_args()
    
    # For now, use the existing simulation function
    # TODO: Update ins_sim.py to accept these parameters
    print(f"Generating {args.trajectory} trajectory:")
    print(f"  Duration: {args.duration}s")
    print(f"  Velocity: {args.velocity} m/s") 
    print(f"  Altitude: {args.altitude} m")
    print(f"  IMU Quality: {args.imu_quality}")
    print()
    
    imu_csv, truth_csv = simulate_straight_and_level()
    print("Generated files:")
    print(f"  IMU data: {imu_csv}")
    print(f"  Truth data: {truth_csv}")

if __name__ == "__main__":
    main()
