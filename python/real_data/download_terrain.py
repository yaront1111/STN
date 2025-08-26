#!/usr/bin/env python3
"""
Download and process SRTM terrain data for real-world navigation
"""

import os
import urllib.request
import zipfile
from pathlib import Path
import struct
import numpy as np

class SRTMDownloader:
    def __init__(self, cache_dir="data/terrain"):
        self.cache_dir = Path(cache_dir)
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        
        # SRTM3 (3 arc-second, ~90m resolution) is freely available
        # SRTM1 (1 arc-second, ~30m resolution) requires login
        self.base_url = "https://dds.cr.usgs.gov/srtm/version2_1/SRTM3/"
        
    def get_tile_name(self, lat, lon):
        """Get SRTM tile name for given coordinates"""
        # SRTM tiles are named by their SW corner
        lat_prefix = 'N' if lat >= 0 else 'S'
        lon_prefix = 'E' if lon >= 0 else 'W'
        
        tile_lat = abs(int(lat))
        tile_lon = abs(int(lon))
        
        return f"{lat_prefix}{tile_lat:02d}{lon_prefix}{tile_lon:03d}"
    
    def download_tile(self, lat, lon):
        """Download SRTM tile for given coordinates"""
        tile_name = self.get_tile_name(lat, lon)
        hgt_file = self.cache_dir / f"{tile_name}.hgt"
        
        if hgt_file.exists():
            print(f"Tile {tile_name} already cached")
            return hgt_file
        
        # Determine region (for USGS directory structure)
        region = self.get_region(lat, lon)
        
        # Try to download
        zip_url = f"{self.base_url}{region}/{tile_name}.hgt.zip"
        zip_path = self.cache_dir / f"{tile_name}.hgt.zip"
        
        print(f"Downloading {tile_name} from {zip_url}...")
        try:
            urllib.request.urlretrieve(zip_url, zip_path)
            
            # Extract HGT file
            with zipfile.ZipFile(zip_path, 'r') as zf:
                zf.extractall(self.cache_dir)
            
            # Clean up zip
            zip_path.unlink()
            
            print(f"Downloaded and extracted {tile_name}")
            return hgt_file
            
        except Exception as e:
            print(f"Failed to download {tile_name}: {e}")
            return None
    
    def get_region(self, lat, lon):
        """Determine SRTM region for USGS directory structure"""
        if lat >= -60 and lat < 60:
            if lon >= -180 and lon < -90:
                return "North_America"
            elif lon >= -90 and lon < -30:
                return "South_America"
            elif lon >= -30 and lon < 60:
                if lat >= 35:
                    return "Eurasia"
                else:
                    return "Africa"
            elif lon >= 60 and lon < 180:
                return "Eurasia"
        return "Islands"  # Default for islands and special regions
    
    def read_hgt(self, hgt_file, lat, lon):
        """Read elevation from HGT file at specific coordinates"""
        if not hgt_file or not hgt_file.exists():
            return None
        
        # SRTM3 has 1201x1201 samples (3 arc-second resolution)
        size = 1201
        
        # Calculate indices
        tile_lat = int(lat)
        tile_lon = int(lon)
        
        # Position within tile (0-1)
        lat_pos = lat - tile_lat
        lon_pos = lon - tile_lon
        
        # Convert to pixel indices
        row = int((1.0 - lat_pos) * (size - 1))
        col = int(lon_pos * (size - 1))
        
        # Read elevation (big-endian 16-bit signed integers)
        with open(hgt_file, 'rb') as f:
            f.seek(2 * (row * size + col))
            buf = f.read(2)
            elevation = struct.unpack('>h', buf)[0]
        
        # Handle voids
        if elevation == -32768:
            return None
        
        return elevation
    
    def download_for_euroc(self):
        """Download terrain tiles for EuRoC dataset location (ETH Zurich)"""
        # ETH Zurich coordinates
        lat = 47.37
        lon = 8.55
        
        print(f"Downloading SRTM data for Zurich area ({lat}, {lon})")
        
        # Download main tile and surrounding tiles
        tiles = []
        for dlat in [-1, 0, 1]:
            for dlon in [-1, 0, 1]:
                tile_lat = int(lat) + dlat
                tile_lon = int(lon) + dlon
                hgt_file = self.download_tile(tile_lat, tile_lon)
                if hgt_file:
                    tiles.append(hgt_file)
        
        print(f"Downloaded {len(tiles)} tiles")
        return tiles
    
    def create_elevation_grid(self, lat_min, lat_max, lon_min, lon_max, resolution=0.001):
        """Create elevation grid for an area"""
        lats = np.arange(lat_min, lat_max, resolution)
        lons = np.arange(lon_min, lon_max, resolution)
        
        grid = np.zeros((len(lats), len(lons)))
        
        for i, lat in enumerate(lats):
            for j, lon in enumerate(lons):
                # Find appropriate tile
                tile_name = self.get_tile_name(lat, lon)
                hgt_file = self.cache_dir / f"{tile_name}.hgt"
                
                if hgt_file.exists():
                    elev = self.read_hgt(hgt_file, lat, lon)
                    if elev is not None:
                        grid[i, j] = elev
        
        return grid, lats, lons

def main():
    print("="*60)
    print("SRTM Terrain Downloader for STN")
    print("="*60)
    
    downloader = SRTMDownloader()
    
    # Download tiles for EuRoC dataset location
    tiles = downloader.download_for_euroc()
    
    if tiles:
        # Test reading elevation at ETH Zurich
        lat, lon = 47.3769, 8.5417
        for tile in tiles:
            elev = downloader.read_hgt(tile, lat, lon)
            if elev is not None:
                print(f"Elevation at ETH Zurich ({lat}, {lon}): {elev} meters")
                break
        
        # Create a small elevation grid
        print("\nCreating elevation grid for Zurich area...")
        grid, lats, lons = downloader.create_elevation_grid(
            47.35, 47.40, 8.50, 8.60, resolution=0.001
        )
        
        print(f"Grid shape: {grid.shape}")
        print(f"Elevation range: {grid.min():.1f} to {grid.max():.1f} meters")
        
        # Save grid for C++ use
        np.save("data/terrain/zurich_elevation.npy", grid)
        np.save("data/terrain/zurich_lats.npy", lats)
        np.save("data/terrain/zurich_lons.npy", lons)
        
        print("\nTerrain data ready for navigation!")
    else:
        print("Failed to download terrain data")

if __name__ == "__main__":
    main()