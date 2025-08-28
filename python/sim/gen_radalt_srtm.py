#!/usr/bin/env python3
"""
Generate radar altimeter measurements using real SRTM terrain data
This ensures consistency between simulation and navigation
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import pandas as pd
import numpy as np
from terrain.srtm_reader import SRTMReader, ned_to_latlon

def generate_radalt_from_srtm(truth_file, output_file, noise_std=0.5):
    """
    Generate radar altimeter measurements from truth trajectory over SRTM terrain
    
    Args:
        truth_file: Path to truth CSV (with pn, pe, pd columns)
        output_file: Path to output radalt CSV
        noise_std: Standard deviation of measurement noise (meters)
    """
    # Load truth trajectory
    truth = pd.read_csv(truth_file)
    print(f"Loaded {len(truth)} truth samples")
    
    # Initialize SRTM reader
    reader = SRTMReader("data/terrain")
    
    # Reference point (Zurich)
    ref_lat = 47.4
    ref_lon = 8.5
    
    # Generate radar altimeter measurements
    agl_measurements = []
    terrain_heights = []
    
    for i in range(len(truth)):
        # Convert NED position to lat/lon
        n = truth['pn'].iloc[i]
        e = truth['pe'].iloc[i]
        d = truth['pd'].iloc[i]  # Negative down (altitude is -pd)
        
        lat, lon = ned_to_latlon(n, e, ref_lat, ref_lon)
        
        # Get terrain elevation from SRTM
        terrain_h = reader.get_elevation(lat, lon)
        terrain_heights.append(terrain_h)
        
        # Calculate AGL: altitude above ground level
        # pd is negative down, so -pd is altitude MSL
        altitude_msl = -d
        agl = altitude_msl - terrain_h
        
        # Add radar altimeter noise
        agl_noisy = agl + np.random.normal(0, noise_std)
        agl_measurements.append(agl_noisy)
        
        # Progress update
        if (i + 1) % 1000 == 0:
            print(f"  Processed {i+1}/{len(truth)} samples...")
    
    # Create output dataframe
    radalt = pd.DataFrame()
    radalt['t'] = truth['t']
    radalt['agl'] = agl_measurements
    
    # Save to CSV
    radalt.to_csv(output_file, index=False)
    
    # Print statistics
    print(f"\nGenerated {len(radalt)} radar altimeter measurements")
    print(f"Statistics:")
    print(f"  Mean AGL: {np.mean(agl_measurements):.1f}m")
    print(f"  Min AGL: {np.min(agl_measurements):.1f}m")
    print(f"  Max AGL: {np.max(agl_measurements):.1f}m")
    print(f"  Std AGL: {np.std(agl_measurements):.1f}m")
    print(f"\nTerrain statistics:")
    print(f"  Mean elevation: {np.mean(terrain_heights):.1f}m")
    print(f"  Min elevation: {np.min(terrain_heights):.1f}m")
    print(f"  Max elevation: {np.max(terrain_heights):.1f}m")
    print(f"  Elevation variation: {np.max(terrain_heights) - np.min(terrain_heights):.1f}m")
    
    # Verify altitude consistency
    first_altitude = -truth['pd'].iloc[0]
    last_altitude = -truth['pd'].iloc[-1]
    print(f"\nTrajectory altitude:")
    print(f"  Start: {first_altitude:.1f}m MSL")
    print(f"  End: {last_altitude:.1f}m MSL")
    
    return radalt


if __name__ == "__main__":
    # Generate radar altimeter data for the current simulation
    print("Generating radar altimeter data using real SRTM terrain...")
    print("=" * 60)
    
    # Check if truth file exists
    truth_file = "data/sim_truth.csv"
    if not os.path.exists(truth_file):
        print(f"Error: Truth file {truth_file} not found")
        print("Please run simulation first: python python/sim/run_sim.py")
        sys.exit(1)
    
    # Generate radar altimeter measurements
    output_file = "data/radalt.csv"
    radalt = generate_radalt_from_srtm(truth_file, output_file)
    
    print(f"\nRadar altimeter data saved to: {output_file}")
    print("\nYou can now run the navigation system:")
    print("  ./build/stn_demo data/radalt.csv data/sim_imu.csv data/run_output.csv")