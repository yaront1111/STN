#!/usr/bin/env python3
"""
SRTM terrain reader for Python - matches C++ implementation
Reads SRTM3 .hgt files and provides elevation queries
"""

import numpy as np
import struct
import os
from pathlib import Path

class SRTMReader:
    def __init__(self, data_dir="data/terrain"):
        self.data_dir = Path(data_dir)
        self.tiles = {}  # Cache loaded tiles
        self.size = 1201  # SRTM3 resolution
        
    def _tile_name(self, lat, lon):
        """Generate SRTM tile filename from coordinates"""
        lat_str = f"N{int(lat):02d}" if lat >= 0 else f"S{int(-lat):02d}"
        lon_str = f"E{int(lon):03d}" if lon >= 0 else f"W{int(-lon):03d}"
        return f"{lat_str}{lon_str}.hgt"
    
    def _load_tile(self, lat, lon):
        """Load an SRTM tile into memory"""
        tile_name = self._tile_name(lat, lon)
        if tile_name in self.tiles:
            return self.tiles[tile_name]
        
        filepath = self.data_dir / tile_name
        if not filepath.exists():
            print(f"Warning: SRTM tile {tile_name} not found")
            return None
            
        # Read SRTM data (big-endian 16-bit signed integers)
        with open(filepath, 'rb') as f:
            data = np.fromfile(f, dtype='>i2').reshape((self.size, self.size))
        
        self.tiles[tile_name] = data
        print(f"Loaded SRTM tile {tile_name}")
        return data
    
    def get_elevation(self, lat, lon):
        """Get elevation at given lat/lon using bilinear interpolation"""
        # Determine which tile we need
        tile_lat = int(np.floor(lat))
        tile_lon = int(np.floor(lon))
        
        # Load the tile
        tile = self._load_tile(tile_lat, tile_lon)
        if tile is None:
            return 0.0  # Default elevation if tile missing
        
        # Calculate position within tile (0-1)
        lat_frac = lat - tile_lat
        lon_frac = lon - tile_lon
        
        # Convert to pixel indices (0-1200)
        # SRTM tiles go from top-left (NW) to bottom-right (SE)
        # Row 0 is at the northern edge, row 1200 is at the southern edge
        row = (1.0 - lat_frac) * (self.size - 1)
        col = lon_frac * (self.size - 1)
        
        # Get integer indices and fractions for interpolation
        row_int = int(row)
        col_int = int(col)
        row_frac = row - row_int
        col_frac = col - col_int
        
        # Ensure we don't go out of bounds
        row_int = np.clip(row_int, 0, self.size - 2)
        col_int = np.clip(col_int, 0, self.size - 2)
        
        # Bilinear interpolation
        # Get four corner heights
        h00 = tile[row_int, col_int]
        h01 = tile[row_int, col_int + 1]
        h10 = tile[row_int + 1, col_int]
        h11 = tile[row_int + 1, col_int + 1]
        
        # Interpolate along rows
        h0 = h00 * (1 - col_frac) + h01 * col_frac
        h1 = h10 * (1 - col_frac) + h11 * col_frac
        
        # Interpolate along columns
        h = h0 * (1 - row_frac) + h1 * row_frac
        
        return float(h)
    
    def get_gradient(self, lat, lon):
        """Get terrain gradient at given lat/lon"""
        # Simple finite difference approximation
        delta = 1.0 / (self.size - 1)  # About 3 arc-seconds
        
        # Get elevations at neighboring points
        h_n = self.get_elevation(lat + delta, lon)
        h_s = self.get_elevation(lat - delta, lon)
        h_e = self.get_elevation(lat, lon + delta)
        h_w = self.get_elevation(lat, lon - delta)
        
        # Calculate gradients in meters per meter
        # At this latitude, 1 degree ≈ 111.3km (lat), 76.7km (lon)
        lat_to_m = 111320.0
        lon_to_m = 111320.0 * np.cos(lat * np.pi / 180.0)
        
        dh_dlat = (h_n - h_s) / (2 * delta * lat_to_m)
        dh_dlon = (h_e - h_w) / (2 * delta * lon_to_m)
        
        # Convert to NED gradients
        # North is positive latitude, East is positive longitude
        dh_dn = dh_dlat
        dh_de = dh_dlon
        
        return dh_dn, dh_de


def ned_to_latlon(n, e, ref_lat=47.4, ref_lon=8.5):
    """Convert NED coordinates to lat/lon"""
    lat_to_m = 111320.0
    lon_to_m = 111320.0 * np.cos(ref_lat * np.pi / 180.0)
    
    lat = ref_lat + n / lat_to_m
    lon = ref_lon + e / lon_to_m
    
    return lat, lon


if __name__ == "__main__":
    # Test the SRTM reader
    reader = SRTMReader()
    
    # Test at Zurich coordinates
    lat, lon = 47.4, 8.5
    elevation = reader.get_elevation(lat, lon)
    gradient = reader.get_gradient(lat, lon)
    
    print(f"\nTest at Zurich ({lat}°N, {lon}°E):")
    print(f"  Elevation: {elevation:.1f}m")
    print(f"  Gradient: N={gradient[0]:.5f}, E={gradient[1]:.5f}")
    
    # Test NED conversion
    n, e = 1000, 500  # 1km north, 500m east of reference
    lat2, lon2 = ned_to_latlon(n, e)
    print(f"\nNED ({n}, {e}) -> Lat/Lon ({lat2:.6f}, {lon2:.6f})")
    print(f"  Elevation: {reader.get_elevation(lat2, lon2):.1f}m")