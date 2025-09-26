#pragma once

#include "srtm_reader.h"
#include <unordered_map>
#include <memory>
#include <string>
#include <mutex>
#include <Eigen/Dense>

namespace terrain {

class TerrainService {
public:
    explicit TerrainService(const std::string& data_directory);
    ~TerrainService() = default;

    // Initialize the service (checks directory and reports available tiles)
    bool initialize();

    // Get elevation at a specific coordinate (loads tile if necessary).
    // Returns -32768.0 for void/out-of-bounds.
    double getElevation(double lat, double lon);

    // Get elevation using bilinear interpolation (same void behavior).
    double getElevationBilinear(double lat, double lon);

    // Terrain normal at lat/lon (unit-length, ENU-ish with +Z up).
    Eigen::Vector3d getTerrainNormal(double lat, double lon);

    // Terrain slope (degrees)
    double getTerrainSlope(double lat, double lon);

    // Preload all tiles overlapping a region [min,max] lat/lon (inclusive).
    bool preloadRegion(double min_lat, double max_lat, double min_lon, double max_lon);

    // Clear cache
    void clearCache();

    // Stats
    size_t getLoadedTileCount() const;

    // Get resolution in degrees per pixel for a coordinate
    double resolutionDeg(double lat, double lon);

private:
    // Generate tile name from coordinates (SW corner)
    std::string getTileName(double lat, double lon) const;

    // Load a tile if not already loaded
    bool loadTileIfNeeded(double lat, double lon);

    // Get the tile containing a coordinate
    std::shared_ptr<SRTMReader> getTileForCoordinate(double lat, double lon);

    // Compute dZ/dE (east), dZ/dN (north) in m/m
    Eigen::Vector2d calculateGradient(double lat, double lon);

private:
    std::string data_directory_;
    std::unordered_map<std::string, std::shared_ptr<SRTMReader>> tiles_;
    mutable std::mutex tiles_mutex_;

    // Central-difference step in degrees (~11 m at equator)
    static constexpr double kGradientStep = 1e-4;
};

} // namespace terrain
