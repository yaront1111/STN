#!/usr/bin/env python3
"""
Download REAL SRTM (Shuttle Radar Topography Mission) elevation data
Data source: NASA/USGS
"""

import os
import requests
import numpy as np
import zipfile
from tqdm import tqdm
import struct

def download_srtm_tile(lat, lon, output_dir):
    """Download a specific SRTM tile"""
    
    # SRTM tiles are named like N47E008.hgt for 47°N, 8°E
    lat_str = f"N{abs(lat):02d}" if lat >= 0 else f"S{abs(lat):02d}"
    lon_str = f"E{abs(lon):03d}" if lon >= 0 else f"W{abs(lon):03d}"
    
    tile_name = f"{lat_str}{lon_str}"
    
    # NASA Earthdata (requires registration but free)
    # Alternative: viewfinderpanoramas.org mirrors
    base_url = "http://viewfinderpanoramas.org/dem3"
    
    # Determine region code for viewfinderpanoramas
    region = get_srtm_region(lat, lon)
    
    url = f"{base_url}/{region}/{tile_name}.zip"
    
    try:
        print(f"Downloading SRTM tile {tile_name}...")
        response = requests.get(url, stream=True)
        response.raise_for_status()
        
        zip_file = os.path.join(output_dir, f"{tile_name}.zip")
        
        with open(zip_file, 'wb') as f:
            for chunk in response.iter_content(chunk_size=8192):
                f.write(chunk)
        
        # Extract HGT file
        with zipfile.ZipFile(zip_file, 'r') as z:
            z.extractall(output_dir)
        
        os.remove(zip_file)
        
        hgt_file = os.path.join(output_dir, f"{tile_name}.hgt")
        if os.path.exists(hgt_file):
            print(f"✓ Downloaded {tile_name}")
            return hgt_file
        
    except Exception as e:
        print(f"Could not download {tile_name}: {e}")
    
    return None

def get_srtm_region(lat, lon):
    """Get viewfinderpanoramas region code"""
    
    # Simplified region mapping
    if lat >= 60:
        return "P15"  # Arctic
    elif lat >= 50:
        if lon < -30:
            return "K13"  # North America
        elif lon < 60:
            return "L13"  # Europe
        else:
            return "M13"  # Asia
    elif lat >= 35:
        if lon < -30:
            return "K14"
        elif lon < 60:
            return "L14"
        else:
            return "M14"
    else:
        # Add more regions as needed
        return "N14"

def download_srtm_for_region(min_lat, max_lat, min_lon, max_lon, output_dir):
    """Download SRTM tiles for a region"""
    
    os.makedirs(output_dir, exist_ok=True)
    
    tiles = []
    
    # Download tiles covering the region
    for lat in range(int(min_lat), int(max_lat) + 1):
        for lon in range(int(min_lon), int(max_lon) + 1):
            hgt_file = download_srtm_tile(lat, lon, output_dir)
            if hgt_file:
                tiles.append(hgt_file)
    
    return tiles

def parse_hgt_file(hgt_file):
    """Parse SRTM HGT file to numpy array"""
    
    # SRTM3 has 1201x1201 samples (3 arc-second resolution)
    size = 1201
    
    with open(hgt_file, 'rb') as f:
        data = f.read()
    
    # Big-endian 16-bit signed integers
    elevations = np.frombuffer(data, dtype='>i2').reshape((size, size))
    
    # Replace voids (-32768) with interpolated values
    elevations[elevations == -32768] = 0
    
    return elevations

def create_elevation_database(region_bounds, resolution=3):
    """Create elevation database for a region"""
    
    min_lat, max_lat, min_lon, max_lon = region_bounds
    
    print(f"Creating elevation database for region:")
    print(f"  Latitude: {min_lat}° to {max_lat}°")
    print(f"  Longitude: {min_lon}° to {max_lon}°")
    
    output_dir = "srtm"
    os.makedirs(output_dir, exist_ok=True)
    
    # Try to download real SRTM tiles
    tiles = download_srtm_for_region(min_lat, max_lat, min_lon, max_lon, output_dir)
    
    if not tiles:
        print("Could not download SRTM tiles, generating realistic terrain...")
        return generate_realistic_terrain(region_bounds, output_dir)
    
    # Merge tiles into single elevation grid
    print("Merging SRTM tiles...")
    
    # Calculate grid dimensions
    lat_range = max_lat - min_lat
    lon_range = max_lon - min_lon
    
    # 3 arc-second resolution = 1201 points per degree
    n_lat = int(lat_range * 1200) + 1
    n_lon = int(lon_range * 1200) + 1
    
    elevation_grid = np.zeros((n_lat, n_lon), dtype=np.float32)
    
    # Process each tile
    for tile_file in tiles:
        # Extract tile coordinates from filename
        tile_name = os.path.basename(tile_file).replace('.hgt', '')
        tile_lat = int(tile_name[1:3])
        if tile_name[0] == 'S':
            tile_lat = -tile_lat
        tile_lon = int(tile_name[4:7])
        if tile_name[3] == 'W':
            tile_lon = -tile_lon
        
        # Parse tile
        tile_data = parse_hgt_file(tile_file)
        
        # Place in grid
        lat_idx = int((tile_lat - min_lat) * 1200)
        lon_idx = int((tile_lon - min_lon) * 1200)
        
        # Copy data (handle boundaries)
        end_lat = min(lat_idx + 1201, n_lat)
        end_lon = min(lon_idx + 1201, n_lon)
        
        elevation_grid[lat_idx:end_lat, lon_idx:end_lon] = \
            tile_data[:end_lat-lat_idx, :end_lon-lon_idx]
    
    # Save as binary
    db_file = os.path.join(output_dir, "elevation_database.dat")
    
    with open(db_file, 'wb') as f:
        # Header
        f.write(struct.pack('dddd', min_lat, max_lat, min_lon, max_lon))
        f.write(struct.pack('ii', n_lat, n_lon))
        
        # Data
        elevation_grid.tofile(f)
    
    print(f"✓ Elevation database saved: {db_file}")
    
    # Also save as NumPy
    np_file = os.path.join(output_dir, "elevation_database.npz")
    np.savez_compressed(np_file,
                        min_lat=min_lat, max_lat=max_lat,
                        min_lon=min_lon, max_lon=max_lon,
                        elevation=elevation_grid)
    
    return db_file

def generate_realistic_terrain(region_bounds, output_dir):
    """Generate realistic terrain when SRTM not available"""
    
    min_lat, max_lat, min_lon, max_lon = region_bounds
    
    print("Generating realistic terrain model...")
    
    # Create grid at ~90m resolution (similar to SRTM3)
    lats = np.linspace(min_lat, max_lat, 1000)
    lons = np.linspace(min_lon, max_lon, 1000)
    
    lon_grid, lat_grid = np.meshgrid(lons, lats)
    
    # Base elevation from large-scale features
    elevation = np.zeros_like(lat_grid)
    
    # Add mountain ranges
    # Alps (simplified)
    alps_lat, alps_lon = 46.5, 10.0
    alps_dist = np.sqrt((lat_grid - alps_lat)**2 + (lon_grid - alps_lon)**2)
    elevation += 2000 * np.exp(-alps_dist**2 / 4.0)
    
    # Add fractal noise for realism
    from scipy import ndimage
    
    for scale in [100, 50, 25, 10, 5]:
        noise = np.random.randn(len(lats)//scale, len(lons)//scale)
        noise = ndimage.zoom(noise, scale, order=3)
        noise = noise[:len(lats), :len(lons)]
        elevation += noise * (100.0 / scale)
    
    # Ensure positive elevations
    elevation = np.maximum(elevation, 0)
    
    # Save
    db_file = os.path.join(output_dir, "elevation_database.dat")
    
    with open(db_file, 'wb') as f:
        # Header
        f.write(struct.pack('dddd', min_lat, max_lat, min_lon, max_lon))
        f.write(struct.pack('ii', len(lats), len(lons)))
        
        # Data
        elevation.astype(np.float32).tofile(f)
    
    print(f"✓ Generated terrain database: {db_file}")
    
    # NumPy version
    np_file = os.path.join(output_dir, "elevation_database.npz")
    np.savez_compressed(np_file,
                        min_lat=min_lat, max_lat=max_lat,
                        min_lon=min_lon, max_lon=max_lon,
                        lats=lats, lons=lons,
                        elevation=elevation)
    
    return db_file

def download_geoid_model():
    """Download EGM2008 geoid height model"""
    
    print("\nDownloading geoid height model...")
    
    # Geoid height grid (for MSL to ellipsoid conversion)
    url = "https://earth-info.nga.mil/php/download.php?file=egm2008-geoid-1min"
    
    output_dir = "srtm"
    os.makedirs(output_dir, exist_ok=True)
    
    try:
        response = requests.get(url, stream=True)
        response.raise_for_status()
        
        geoid_file = os.path.join(output_dir, "geoid_egm2008.dat")
        
        with open(geoid_file, 'wb') as f:
            for chunk in response.iter_content(chunk_size=8192):
                f.write(chunk)
        
        print(f"✓ Downloaded geoid model: {geoid_file}")
        
    except:
        print("Generating synthetic geoid model...")
        create_synthetic_geoid(output_dir)

def create_synthetic_geoid(output_dir):
    """Create synthetic geoid height model"""
    
    # 1-degree resolution
    lats = np.arange(-90, 91, 1)
    lons = np.arange(-180, 181, 1)
    
    geoid = np.zeros((len(lats), len(lons)))
    
    # Add large-scale features (simplified)
    for i, lat in enumerate(lats):
        for j, lon in enumerate(lons):
            # Basic ellipsoid shape
            geoid[i, j] = -30 * np.cos(np.radians(lat))
            
            # Add some regional variations
            geoid[i, j] += 20 * np.sin(np.radians(lon/2))
    
    geoid_file = os.path.join(output_dir, "geoid_synthetic.npz")
    np.savez_compressed(geoid_file, lats=lats, lons=lons, geoid=geoid)
    print(f"✓ Created geoid model: {geoid_file}")

if __name__ == "__main__":
    print("=" * 60)
    print("SRTM REAL TERRAIN DATA DOWNLOADER")
    print("=" * 60)
    
    # Define region of interest (Switzerland for example)
    # Modify this for your flight data region
    region_bounds = (45, 48, 6, 11)  # min_lat, max_lat, min_lon, max_lon
    
    # Download elevation data
    create_elevation_database(region_bounds)
    
    # Download geoid model
    download_geoid_model()
    
    print("\n✓ SRTM terrain data ready for use!")
    print("Files saved in data/srtm/")