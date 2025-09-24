/**
 * SRTM Terrain Map Manager
 * Provides terrain elevation data from SRTM tiles
 * Supports 1 and 3 arc-second resolution
 */

#pragma once

#include "map_manager.h"
#include <vector>

namespace Navigation {

/**
 * Terrain query result
 */
struct TerrainQueryResult : public MapQueryResult {
    double elevation;           // meters above sea level
    double slope;              // degrees
    double aspect;             // degrees from north
    Vector3d surface_normal;   // Unit normal vector
    bool is_water = false;     // Water body flag
    bool is_void = false;      // Data void flag
};

/**
 * SRTM configuration
 */
struct SRTMConfig {
    std::string data_directory;   // Path to SRTM .hgt files
    int resolution = 3;           // Arc-seconds (1 or 3)
    bool interpolate = true;      // Bilinear interpolation
    bool fill_voids = true;       // Fill data voids
    double void_fill_radius = 10; // Pixels for void filling
    double water_level = 0.0;     // Sea level elevation
};

/**
 * SRTM tile data
 */
struct SRTMTile : public MapTile {
    int samples_per_line;      // 1201 for 3", 3601 for 1"
    int num_lines;            // Same as samples_per_line
    std::vector<int16_t> elevations;  // Raw elevation data
    bool has_voids = false;
    int void_count = 0;
};

/**
 * SRTM Terrain Map Manager
 */
class SRTMTerrain : public MapManager {
private:
    SRTMConfig config_;
    
    // Constants
    static constexpr int16_t SRTM_VOID = -32768;  // Void flag value
    static constexpr int SRTM3_SIZE = 1201;        // 3 arc-second tile size
    static constexpr int SRTM1_SIZE = 3601;        // 1 arc-second tile size
    
public:
    SRTMTerrain(const std::string& data_path, const SRTMConfig& config);
    ~SRTMTerrain() = default;
    
    // MapManager interface
    bool initialize() override;
    MapQueryResult query(double latitude, double longitude, double altitude = 0) const override;
    bool isAvailable(double latitude, double longitude) const override;
    
    // Terrain-specific queries
    TerrainQueryResult queryTerrain(double lat, double lon) const;
    double getElevation(double lat, double lon) const;
    Vector3d getTerrainGradient(double lat, double lon) const;
    Vector3d getSurfaceNormal(double lat, double lon) const;
    double getSlope(double lat, double lon) const;
    double getAspect(double lat, double lon) const;
    
    // Simplified interface for navigation (x,y in meters)
    double getElevationXY(double x_meters, double y_meters) const {
        // Convert from meters to lat/lon (simplified)
        double lat = x_meters / 111111.0;
        double lon = y_meters / (111111.0 * cos(lat * M_PI / 180.0));
        return getElevation(lat, lon);
    }
    
protected:
    // MapManager tile loading
    std::shared_ptr<MapTile> loadTile(int tile_x, int tile_y) const override;
    
private:
    // Load SRTM .hgt file
    std::shared_ptr<SRTMTile> loadSRTMFile(double lat, double lon) const;
    std::string getSRTMFilename(int lat, int lon) const;
    
    // Parse binary SRTM data
    bool parseSRTMData(const std::string& filename, SRTMTile& tile) const;
    int16_t swapEndian(int16_t value) const;
    
    // Elevation queries
    double getElevationFromTile(const SRTMTile& tile, double lat, double lon) const;
    double interpolateElevation(const SRTMTile& tile, double x, double y) const;
    
    // Void filling
    void fillVoids(SRTMTile& tile) const;
    double averageNearbyElevations(const SRTMTile& tile, int x, int y, int radius) const;
    
    // Gradient and normal computation
    Vector3d computeGradient(const SRTMTile& tile, double x, double y) const;
    Vector3d computeSurfaceNormal(const Vector3d& gradient) const;
    
    // Utility functions
    void pixelToGeo(int x, int y, int tile_lat, int tile_lon, 
                   double& lat, double& lon) const;
    void geoToPixel(double lat, double lon, int tile_lat, int tile_lon,
                   double& x, double& y) const;
    bool isWater(double elevation) const;
};

/**
 * Factory specialization for SRTM
 */
class TerrainMapManager : public SRTMTerrain {
public:
    TerrainMapManager(const std::string& data_path)
        : SRTMTerrain(data_path, SRTMConfig{}) {}
};

} // namespace Navigation