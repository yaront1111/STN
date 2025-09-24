/**
 * Map Manager Base Implementation
 * Common functionality for all map types
 */

#include "map_manager.h"
#include <algorithm>
#include <cmath>
#include <sstream>

namespace Navigation {

MapManager::MapManager(const YAML::Node& config) {
    // Parse YAML config
    map_name_ = config["name"].as<std::string>("CompositeMap");
    data_path_ = config["data_path"].as<std::string>("data/maps");
    resolution_ = config["resolution"].as<double>(1.0);

    // Parse bounds if provided
    if (config["bounds"]) {
        global_bounds_.lat_min = config["bounds"]["lat_min"].as<double>(-90);
        global_bounds_.lat_max = config["bounds"]["lat_max"].as<double>(90);
        global_bounds_.lon_min = config["bounds"]["lon_min"].as<double>(-180);
        global_bounds_.lon_max = config["bounds"]["lon_max"].as<double>(180);
    } else {
        global_bounds_.lat_min = -90;
        global_bounds_.lat_max = 90;
        global_bounds_.lon_min = -180;
        global_bounds_.lon_max = 180;
    }

    if (config["cache_size"]) {
        max_cache_size_ = config["cache_size"].as<size_t>(100);
    }

    {


        std::stringstream msg;


        msg << "MapManager '" << map_name_ << "' initialized from YAML with data path: " << data_path_;


        LOG_INFO(msg.str());


    }
}

MapManager::MapManager(const std::string& name, const std::string& data_path)
    : map_name_(name), data_path_(data_path) {
    {

        std::stringstream msg;

        msg << "Map Manager initialized: " << name << " at " << data_path;

        LOG_INFO(msg.str());

    }
}

void MapManager::clearCache() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    tile_cache_.clear();
    {

        std::stringstream msg;

        msg << "Cache cleared for " << map_name_;

        LOG_INFO(msg.str());

    }
}

size_t MapManager::getCacheSize() const {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return tile_cache_.size();
}

double MapManager::getCacheHitRate() const {
    if (queries_ == 0) return 0.0;
    return static_cast<double>(cache_hits_) / queries_;
}

void MapManager::printStatistics() const {
    {

        std::stringstream msg;

        msg << "=== Map Manager Statistics: " << map_name_ << " ===";

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "Total queries: " << queries_;

        LOG_INFO(msg.str());

    }
    {
        std::stringstream msg;
        msg << "Cache hits: " << cache_hits_ << " ("
             << std::fixed << std::setprecision(1)
             << getCacheHitRate() * 100 << "%)";
        LOG_INFO(msg.str());
    }
    {

        std::stringstream msg;

        msg << "Cache misses: " << cache_misses_;

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "Current cache size: " << getCacheSize() << " / " << max_cache_size_;

        LOG_INFO(msg.str());

    }
}

void MapManager::resetStatistics() {
    queries_ = 0;
    cache_hits_ = 0;
    cache_misses_ = 0;
}

std::shared_ptr<MapTile> MapManager::getTile(int tile_x, int tile_y) const {
    std::string key = std::to_string(tile_x) + "_" + std::to_string(tile_y);
    
    std::lock_guard<std::mutex> lock(cache_mutex_);
    queries_++;
    
    // Check cache
    auto it = tile_cache_.find(key);
    if (it != tile_cache_.end()) {
        cache_hits_++;
        it->second->access_count++;
        it->second->last_access = std::chrono::steady_clock::now();
        return it->second;
    }
    
    // Cache miss - load tile
    cache_misses_++;
    {

        std::stringstream msg;

        msg << "Cache miss for tile " << key << ", loading...";

        LOG_DEBUG(msg.str());

    }
    
    auto tile = loadTile(tile_x, tile_y);
    if (tile) {
        addToCache(tile);
    }
    
    return tile;
}

void MapManager::addToCache(std::shared_ptr<MapTile> tile) const {
    if (!tile) return;
    
    // Check cache size
    if (tile_cache_.size() >= max_cache_size_) {
        evictLRU();
    }
    
    tile->last_access = std::chrono::steady_clock::now();
    tile_cache_[tile->getKey()] = tile;
}

void MapManager::evictLRU() const {
    if (tile_cache_.empty()) return;
    
    // Find least recently used tile
    auto lru_it = tile_cache_.begin();
    auto oldest_time = lru_it->second->last_access;
    
    for (auto it = tile_cache_.begin(); it != tile_cache_.end(); ++it) {
        if (it->second->last_access < oldest_time) {
            oldest_time = it->second->last_access;
            lru_it = it;
        }
    }
    
    {

    
        std::stringstream msg;

    
        msg << "Evicting tile " << lru_it->first << " from cache";

    
        LOG_DEBUG(msg.str());

    
    }
    tile_cache_.erase(lru_it);
}

void MapManager::geoToTileIndex(double lat, double lon, int& tile_x, int& tile_y) const {
    // Convert geographic coordinates to tile indices
    // This is a simple implementation - derived classes may override
    double tiles_per_degree = 1.0 / resolution_;  // Assuming resolution in degrees
    
    tile_x = static_cast<int>((lon - global_bounds_.lon_min) * tiles_per_degree);
    tile_y = static_cast<int>((lat - global_bounds_.lat_min) * tiles_per_degree);
}

void MapManager::tileIndexToGeo(int tile_x, int tile_y, double& lat, double& lon) const {
    // Convert tile indices back to geographic coordinates (tile center)
    double tiles_per_degree = 1.0 / resolution_;
    
    lon = global_bounds_.lon_min + (tile_x + 0.5) / tiles_per_degree;
    lat = global_bounds_.lat_min + (tile_y + 0.5) / tiles_per_degree;
}

double MapManager::bilinearInterpolate(const MatrixXd& data, double x, double y) const {
    // Bilinear interpolation
    int x0 = static_cast<int>(x);
    int y0 = static_cast<int>(y);
    int x1 = x0 + 1;
    int y1 = y0 + 1;
    
    // Bounds checking
    if (x0 < 0 || y0 < 0 || x1 >= data.cols() || y1 >= data.rows()) {
        // Nearest neighbor for edge cases
        int xi = std::max(0, std::min(static_cast<int>(x), static_cast<int>(data.cols() - 1)));
        int yi = std::max(0, std::min(static_cast<int>(y), static_cast<int>(data.rows() - 1)));
        return data(yi, xi);
    }
    
    // Fractional parts
    double fx = x - x0;
    double fy = y - y0;
    
    // Bilinear interpolation
    double v00 = data(y0, x0);
    double v10 = data(y0, x1);
    double v01 = data(y1, x0);
    double v11 = data(y1, x1);
    
    double v0 = v00 * (1 - fx) + v10 * fx;
    double v1 = v01 * (1 - fx) + v11 * fx;
    
    return v0 * (1 - fy) + v1 * fy;
}

Vector3d MapManager::computeGradient(const MatrixXd& data, double x, double y, double scale) const {
    // Compute gradient using finite differences
    Vector3d gradient;
    
    int xi = static_cast<int>(x);
    int yi = static_cast<int>(y);
    
    // Bounds checking
    if (xi <= 0 || yi <= 0 || xi >= data.cols() - 1 || yi >= data.rows() - 1) {
        return Vector3d::Zero();  // No gradient at boundaries
    }
    
    // Central differences
    double dx = (data(yi, xi + 1) - data(yi, xi - 1)) / (2.0 * scale);
    double dy = (data(yi + 1, xi) - data(yi - 1, xi)) / (2.0 * scale);
    
    gradient << dx, dy, 0;
    
    return gradient;
}

// Factory implementation
std::unique_ptr<MapManager> MapManagerFactory::create(const std::string& type,
                                                      const std::string& data_path) {
    // This will be expanded when we implement specific map types
    {

        std::stringstream msg;

        msg << "MapManagerFactory not yet fully implemented for type: " << type;

        LOG_ERROR(msg.str());

    }
    return nullptr;
}

} // namespace Navigation