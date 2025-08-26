#!/usr/bin/env python3
"""
Download/Create EGM2008 gravity model data
"""

import numpy as np
import struct
from pathlib import Path

def create_egm2008_grid(output_file="../../data/egm2008.dat"):
    """Create simplified EGM2008 gravity anomaly grid"""
    
    print("="*60)
    print("Creating EGM2008 Gravity Model Grid")
    print("="*60)
    
    # Create output directory
    Path(output_file).parent.mkdir(parents=True, exist_ok=True)
    
    # Grid parameters (simplified 1-degree grid)
    lat_min, lat_max = -90, 90
    lon_min, lon_max = -180, 180
    resolution = 1.0  # degrees
    
    n_lat = int((lat_max - lat_min) / resolution) + 1
    n_lon = int((lon_max - lon_min) / resolution) + 1
    
    print(f"Grid size: {n_lat} x {n_lon}")
    print(f"Resolution: {resolution} degrees")
    
    # Generate realistic gravity anomaly patterns
    # Real EGM2008 ranges from about -100 to +100 mGal
    anomalies = np.zeros((n_lat, n_lon), dtype=np.float32)
    
    for i in range(n_lat):
        lat = lat_min + i * resolution
        for j in range(n_lon):
            lon = lon_min + j * resolution
            
            # Continental vs oceanic (rough approximation)
            is_ocean = False
            if abs(lat) < 60:  # Not polar
                # Atlantic
                if -60 < lon < -10 and abs(lat) < 45:
                    is_ocean = True
                # Pacific
                if (lon < -120 or lon > 150) and abs(lat) < 45:
                    is_ocean = True
                # Indian Ocean
                if 40 < lon < 120 and -40 < lat < 20:
                    is_ocean = True
            
            # Base anomaly
            if is_ocean:
                base = -10.0  # Oceanic crust is denser
            else:
                base = 5.0    # Continental crust
            
            # Add features
            # Mountain ranges (Alps, Himalayas, Andes, Rockies)
            if 45 < lat < 48 and 6 < lon < 15:  # Alps
                base += 30 * np.exp(-0.1 * ((lat-47)**2 + (lon-10)**2))
            if 25 < lat < 35 and 75 < lon < 95:  # Himalayas
                base += 50 * np.exp(-0.05 * ((lat-30)**2 + (lon-85)**2))
            if -50 < lat < -15 and -80 < lon < -60:  # Andes
                base += 40 * np.exp(-0.05 * ((lat+30)**2 + (lon+70)**2))
            if 35 < lat < 50 and -120 < lon < -100:  # Rockies
                base += 25 * np.exp(-0.05 * ((lat-42)**2 + (lon+110)**2))
            
            # Subduction zones (negative anomalies)
            if 35 < lat < 45 and 140 < lon < 145:  # Japan Trench
                base -= 40 * np.exp(-0.2 * ((lat-40)**2 + (lon-142)**2))
            
            # Add some noise
            base += np.random.normal(0, 5)
            
            # Clamp to realistic range
            anomalies[i, j] = np.clip(base, -100, 100)
    
    # Write binary file
    with open(output_file, 'wb') as f:
        # Header
        f.write(struct.pack('i', n_lat))      # Number of latitude points
        f.write(struct.pack('i', n_lon))      # Number of longitude points
        f.write(struct.pack('f', lat_min))    # Minimum latitude
        f.write(struct.pack('f', lat_max))    # Maximum latitude
        f.write(struct.pack('f', lon_min))    # Minimum longitude
        f.write(struct.pack('f', lon_max))    # Maximum longitude
        f.write(struct.pack('f', resolution)) # Grid resolution
        
        # Data (row-major order, starting from south)
        for i in range(n_lat):
            for j in range(n_lon):
                f.write(struct.pack('f', anomalies[i, j]))
    
    file_size = Path(output_file).stat().st_size / 1024
    print(f"\nCreated EGM2008 grid file: {output_file}")
    print(f"File size: {file_size:.1f} KB")
    print(f"Anomaly range: {anomalies.min():.1f} to {anomalies.max():.1f} mGal")
    
    # Show sample values for verification
    print("\nSample values:")
    print(f"  Zurich (47.4°N, 8.5°E): {anomalies[137, 188]:.1f} mGal")
    print(f"  Tokyo (35.7°N, 139.7°E): {anomalies[125, 320]:.1f} mGal")
    print(f"  New York (40.7°N, -74.0°W): {anomalies[130, 106]:.1f} mGal")
    
    print("\n" + "="*60)
    print("EGM2008 gravity model ready for use!")
    print("="*60)

if __name__ == "__main__":
    create_egm2008_grid()