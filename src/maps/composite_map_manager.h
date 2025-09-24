/**
 * Composite Map Manager
 * Manages both gravity and terrain maps
 */

#pragma once

#include "map_manager.h"
#include "xgm2019e_map.h"
#include "srtm_terrain.h"
#include <memory>

namespace Navigation {

class CompositeMapManager : public MapManager {
private:
    std::unique_ptr<GravityMapManager> gravity_map_;
    std::unique_ptr<TerrainMapManager> terrain_map_;
    bool initialized_ = false;

public:
    CompositeMapManager(const YAML::Node& config);
    virtual ~CompositeMapManager() = default;

    // Override pure virtual methods
    virtual bool initialize() override;
    virtual MapQueryResult query(double latitude, double longitude, double altitude = 0) const override;
    virtual bool isAvailable(double latitude, double longitude) const override;

    // Override validation
    virtual bool validateMaps() const override;

    // Access to individual maps (for RBPF)
    GravityMapManager* getGravityMap() const { return gravity_map_.get(); }
    TerrainMapManager* getTerrainMap() const { return terrain_map_.get(); }

protected:
    // Implementation of tile loading (for cache)
    virtual std::shared_ptr<MapTile> loadTile(int tile_x, int tile_y) const override;
};

} // namespace Navigation