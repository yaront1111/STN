#pragma once
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <iostream>
#include <map>

// SRTM HGT file reader for real terrain elevation
class SRTMReader {
private:
    struct TileData {
        std::vector<int16_t> elevations;
        int size;  // 1201 for SRTM3, 3601 for SRTM1
        bool loaded;
    };
    
    std::map<std::string, TileData> tiles_;
    std::string data_path_;
    
    // Get tile name from coordinates
    std::string getTileName(double lat, double lon) const {
        int lat_int = static_cast<int>(std::floor(lat));
        int lon_int = static_cast<int>(std::floor(lon));
        
        char ns = (lat_int >= 0) ? 'N' : 'S';
        char ew = (lon_int >= 0) ? 'E' : 'W';
        
        char tile_name[12];
        snprintf(tile_name, sizeof(tile_name), "%c%02d%c%03d", 
                 ns, std::abs(lat_int), ew, std::abs(lon_int));
        
        return std::string(tile_name);
    }
    
    // Load HGT file
    bool loadTile(const std::string& tile_name) {
        if (tiles_.find(tile_name) != tiles_.end() && tiles_[tile_name].loaded) {
            return true;  // Already loaded
        }
        
        std::string hgt_path = data_path_ + "/" + tile_name + ".hgt";
        std::ifstream file(hgt_path, std::ios::binary);
        
        if (!file.is_open()) {
            std::cerr << "Failed to open SRTM tile: " << hgt_path << std::endl;
            return false;
        }
        
        // Determine file size to detect SRTM1 vs SRTM3
        file.seekg(0, std::ios::end);
        size_t file_size = file.tellg();
        file.seekg(0, std::ios::beg);
        
        int size;
        if (file_size == 1201 * 1201 * 2) {
            size = 1201;  // SRTM3 (3 arc-second)
        } else if (file_size == 3601 * 3601 * 2) {
            size = 3601;  // SRTM1 (1 arc-second)
        } else {
            std::cerr << "Invalid SRTM file size: " << file_size << std::endl;
            return false;
        }
        
        // Read elevation data (big-endian 16-bit signed integers)
        TileData& tile = tiles_[tile_name];
        tile.size = size;
        tile.elevations.resize(size * size);
        
        for (int i = 0; i < size * size; i++) {
            uint8_t bytes[2];
            file.read(reinterpret_cast<char*>(bytes), 2);
            
            // Convert from big-endian to host byte order
            int16_t elevation = (bytes[0] << 8) | bytes[1];
            tile.elevations[i] = elevation;
        }
        
        tile.loaded = true;
        std::cout << "Loaded SRTM tile " << tile_name << " (" << size << "x" << size << ")" << std::endl;
        
        return true;
    }
    
public:
    explicit SRTMReader(const std::string& data_path = "data/terrain") 
        : data_path_(data_path) {}
    
    // Get elevation at specific coordinates
    double getElevation(double lat, double lon) {
        std::string tile_name = getTileName(lat, lon);
        
        // Load tile if not already loaded
        if (!loadTile(tile_name)) {
            // If real SRTM not available, use synthetic terrain
            return getSyntheticElevation(lat, lon);
        }
        
        const TileData& tile = tiles_[tile_name];
        double resolution = (tile.size == 1201) ? (1.0 / 1200.0) : (1.0 / 3600.0);
        
        // Calculate position within tile
        double tile_lat = std::floor(lat);
        double tile_lon = std::floor(lon);
        
        double lat_offset = lat - tile_lat;
        double lon_offset = lon - tile_lon;
        
        // Convert to pixel indices (origin is NW corner)
        double row_f = (1.0 - lat_offset) / resolution;
        double col_f = lon_offset / resolution;
        
        int row = static_cast<int>(row_f);
        int col = static_cast<int>(col_f);
        
        // Bounds checking
        row = std::max(0, std::min(tile.size - 2, row));
        col = std::max(0, std::min(tile.size - 2, col));
        
        // Bilinear interpolation
        double fx = col_f - col;
        double fy = row_f - row;
        
        int idx00 = row * tile.size + col;
        int idx01 = row * tile.size + (col + 1);
        int idx10 = (row + 1) * tile.size + col;
        int idx11 = (row + 1) * tile.size + (col + 1);
        
        double h00 = tile.elevations[idx00];
        double h01 = tile.elevations[idx01];
        double h10 = tile.elevations[idx10];
        double h11 = tile.elevations[idx11];
        
        // Handle voids (SRTM uses -32768 for no data)
        if (h00 == -32768) h00 = 0;
        if (h01 == -32768) h01 = 0;
        if (h10 == -32768) h10 = 0;
        if (h11 == -32768) h11 = 0;
        
        double elevation = h00 * (1 - fx) * (1 - fy) +
                          h01 * fx * (1 - fy) +
                          h10 * (1 - fx) * fy +
                          h11 * fx * fy;
        
        return elevation;
    }
    
    // Get terrain gradient at location
    Eigen::Vector2d getGradient(double lat, double lon) {
        const double delta = 0.00001;  // ~1 meter at equator
        
        double h_north = getElevation(lat + delta, lon);
        double h_south = getElevation(lat - delta, lon);
        double h_east = getElevation(lat, lon + delta);
        double h_west = getElevation(lat, lon - delta);
        
        // Convert to meters per meter
        const double lat_to_m = 111320.0;
        const double lon_to_m = 111320.0 * cos(lat * M_PI / 180.0);
        
        double dh_dlat = (h_north - h_south) / (2 * delta * lat_to_m);
        double dh_dlon = (h_east - h_west) / (2 * delta * lon_to_m);
        
        return Eigen::Vector2d(dh_dlat, dh_dlon);
    }
    
    // Fallback synthetic terrain for testing
    double getSyntheticElevation(double lat, double lon) {
        // Create realistic terrain around Zurich
        double base_elevation = 400.0;  // Zurich average elevation
        
        // Add hills and valleys
        double terrain = base_elevation;
        terrain += 50 * sin(lat * 10) * cos(lon * 10);  // Rolling hills
        terrain += 100 * exp(-0.1 * ((lat - 47.4) * (lat - 47.4) + 
                                     (lon - 8.5) * (lon - 8.5)));  // Mountain near Zurich
        
        return terrain;
    }
};