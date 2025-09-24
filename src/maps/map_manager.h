/**
 * Map Manager Base Class
 * Abstract interface for all map types
 * Thread-safe tile-based caching
 */

#pragma once

#include <Eigen/Dense>
#include <memory>
#include <unordered_map>
#include <mutex>
#include <string>
#include <yaml-cpp/yaml.h>
#include "../utils/logger.h"
#include "../utils/math_utils.h"

namespace Navigation {

using namespace NavMath;

/**
 * Geographic bounds
 */
struct GeoBounds {
    double lat_min;
    double lat_max;
    double lon_min;
    double lon_max;
    
    bool contains(double lat, double lon) const {
        return lat >= lat_min && lat <= lat_max &&
               lon >= lon_min && lon <= lon_max;
    }
    
    double area() const {
        return (lat_max - lat_min) * (lon_max - lon_min);
    }
};

/**
 * Map tile for caching
 */
struct MapTile {
    int tile_x;           // Tile index X
    int tile_y;           // Tile index Y
    GeoBounds bounds;     // Geographic bounds
    MatrixXd data;        // Tile data
    double resolution;    // Data resolution (degrees or meters)
    uint64_t access_count = 0;
    std::chrono::steady_clock::time_point last_access;
    
    std::string getKey() const {
        return std::to_string(tile_x) + "_" + std::to_string(tile_y);
    }
};

/**
 * Map query result
 */
struct MapQueryResult {
    bool valid = false;
    double value = 0.0;
    Vector3d gradient = Vector3d::Zero();
    double confidence = 1.0;
    std::string source_tile;
};

/**
 * Base class for all map managers
 */
class MapManager {
protected:
    // Map metadata
    std::string map_name_;
    std::string data_path_;
    GeoBounds global_bounds_;
    double resolution_;  // Arc-seconds or meters
    
    // Tile cache (thread-safe)
    mutable std::unordered_map<std::string, std::shared_ptr<MapTile>> tile_cache_;
    mutable std::mutex cache_mutex_;
    size_t max_cache_size_ = 100;  // Maximum tiles in cache
    
    // Statistics
    mutable uint64_t queries_ = 0;
    mutable uint64_t cache_hits_ = 0;
    mutable uint64_t cache_misses_ = 0;
    
public:
    MapManager(const std::string& name, const std::string& data_path);
    MapManager(const YAML::Node& config);  // YAML constructor
    virtual ~MapManager() = default;
    
    // Pure virtual - must be implemented by derived classes
    virtual bool initialize() = 0;
    virtual MapQueryResult query(double latitude, double longitude, double altitude = 0) const = 0;
    virtual bool isAvailable(double latitude, double longitude) const = 0;
    
    // Common functionality
    GeoBounds getBounds() const { return global_bounds_; }
    double getResolution() const { return resolution_; }
    std::string getName() const { return map_name_; }

    // Validation
    virtual bool validateMaps() const {
        // Basic validation - derived classes can override for specific checks
        return !data_path_.empty() && global_bounds_.area() > 0;
    }
    
    // Cache management
    void clearCache();
    size_t getCacheSize() const;
    double getCacheHitRate() const;
    void setCacheSize(size_t max_tiles) { max_cache_size_ = max_tiles; }
    
    // Statistics
    void printStatistics() const;
    void resetStatistics();
    
protected:
    // Tile management (for derived classes)
    std::shared_ptr<MapTile> getTile(int tile_x, int tile_y) const;
    virtual std::shared_ptr<MapTile> loadTile(int tile_x, int tile_y) const = 0;
    void addToCache(std::shared_ptr<MapTile> tile) const;
    void evictLRU() const;  // Least Recently Used eviction
    
    // Coordinate conversions
    void geoToTileIndex(double lat, double lon, int& tile_x, int& tile_y) const;
    void tileIndexToGeo(int tile_x, int tile_y, double& lat, double& lon) const;
    
    // Interpolation helpers
    double bilinearInterpolate(const MatrixXd& data, double x, double y) const;
    Vector3d computeGradient(const MatrixXd& data, double x, double y, double scale) const;
};

/**
 * Factory for creating map managers
 */
class MapManagerFactory {
public:
    static std::unique_ptr<MapManager> create(const std::string& type,
                                              const std::string& data_path);
};

// Forward declarations for specific map types
class GravityMapManager;
class TerrainMapManager;

} // namespace Navigation