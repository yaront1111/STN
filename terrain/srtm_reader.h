#pragma once

#include <string>
#include <vector>
#include <cstdint>

namespace terrain {

class SRTMReader {
public:
    SRTMReader();
    ~SRTMReader() = default;

    // Load an SRTM HGT file
    bool loadTile(const std::string& filepath);

    // Get elevation at specific coordinates (meters MSL).
    // Returns -32768.0 for void/out-of-bounds (SRTM void sentinel).
    double getElevation(double lat, double lon) const;

    // Bilinear interpolation (meters MSL). Same void behavior as above.
    double getElevationBilinear(double lat, double lon) const;

    // Check if a coordinate is within the loaded tile
    bool containsCoordinate(double lat, double lon) const;

    // Tile boundaries
    double getMinLat() const { return min_lat_; }
    double getMaxLat() const { return max_lat_; }
    double getMinLon() const { return min_lon_; }
    double getMaxLon() const { return max_lon_; }

    // Tile size (1201 for SRTM3, 3601 for SRTM1)
    int getSize() const { return size_; }

    // Is tile loaded?
    bool isLoaded() const { return loaded_; }

private:
    // Parse tile name (e.g., "N32E034.hgt") -> SW corner
    static bool parseTileName(const std::string& filename,
                              double& sw_lat_deg,
                              double& sw_lon_deg);

    // Map lat/lon to pixel indices (row, col). Row 0 = north edge, Col 0 = west edge.
    void getPixelIndices(double lat, double lon, int& row, int& col) const;

    // Continuous indices (fractional row/col for interpolation)
    void getPixelIndicesFrac(double lat, double lon, double& r, double& c) const;

    // Helpers
    static inline bool isVoid(int16_t h) { return h == INT16_MIN; } // -32768

private:
    std::vector<int16_t> elevation_data_; // big-endian converted to host, in meters (int16)
    int size_ = 0;                         // 1201 or 3601
    double min_lat_ = 0.0, max_lat_ = 0.0; // degrees
    double min_lon_ = 0.0, max_lon_ = 0.0; // degrees
    bool loaded_ = false;
    std::string tile_name_;
};

} // namespace terrain
