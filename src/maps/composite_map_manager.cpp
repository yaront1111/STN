/**
 * Composite Map Manager Implementation
 */

#include "composite_map_manager.h"
#include "../utils/logger.h"
#include <sstream>

namespace Navigation {

CompositeMapManager::CompositeMapManager(const YAML::Node& config)
    : MapManager(config) {

    // Create gravity map if config exists
    if (config["gravity"] && config["gravity"]["data_path"]) {
        std::string gravity_path = config["gravity"]["data_path"].as<std::string>();
        gravity_map_ = std::make_unique<GravityMapManager>(gravity_path);
    }

    // Create terrain map if config exists
    if (config["terrain"] && config["terrain"]["data_path"]) {
        std::string terrain_path = config["terrain"]["data_path"].as<std::string>();
        terrain_map_ = std::make_unique<TerrainMapManager>(terrain_path);
    }

    LOG_INFO("CompositeMapManager created with gravity and terrain maps");
}

bool CompositeMapManager::initialize() {
    bool success = true;

    if (gravity_map_) {
        if (!gravity_map_->initialize()) {
            LOG_ERROR("Failed to initialize gravity map");
            success = false;
        }
    }

    if (terrain_map_) {
        if (!terrain_map_->initialize()) {
            LOG_ERROR("Failed to initialize terrain map");
            success = false;
        }
    }

    if (!gravity_map_ && !terrain_map_) {
        LOG_ERROR("No maps configured in CompositeMapManager");
        success = false;
    }

    initialized_ = success;
    return success;
}

MapQueryResult CompositeMapManager::query(double latitude, double longitude, double altitude) const {
    MapQueryResult result;

    // Query gravity map if available
    if (gravity_map_ && gravity_map_->isAvailable(latitude, longitude)) {
        auto gravity_result = gravity_map_->query(latitude, longitude, altitude);
        if (gravity_result.valid) {
            result = gravity_result;
        }
    }

    // Also query terrain if available (could combine results)
    if (terrain_map_ && terrain_map_->isAvailable(latitude, longitude)) {
        auto terrain_result = terrain_map_->query(latitude, longitude, altitude);
        if (terrain_result.valid && !result.valid) {
            result = terrain_result;
        }
    }

    return result;
}

bool CompositeMapManager::isAvailable(double latitude, double longitude) const {
    bool available = false;

    if (gravity_map_) {
        available |= gravity_map_->isAvailable(latitude, longitude);
    }

    if (terrain_map_) {
        available |= terrain_map_->isAvailable(latitude, longitude);
    }

    return available;
}

bool CompositeMapManager::validateMaps() const {
    if (!initialized_) {
        return false;
    }

    bool valid = true;

    if (gravity_map_) {
        valid &= gravity_map_->validateMaps();
    }

    if (terrain_map_) {
        valid &= terrain_map_->validateMaps();
    }

    return valid;
}

std::shared_ptr<MapTile> CompositeMapManager::loadTile(int tile_x, int tile_y) const {
    // For composite map, we don't load tiles directly
    // Individual maps handle their own tiles
    return nullptr;
}

} // namespace Navigation