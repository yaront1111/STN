#include "terrain_service.h"
#include "core/utils/logging.h"

#include <filesystem>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cctype>
#include <cmath>

namespace terrain {

static inline bool has_hgt_ext_ci(const std::filesystem::path& p) {
    auto ext = p.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c){ return std::tolower(c); });
    return ext == ".hgt";
}

TerrainService::TerrainService(const std::string& data_directory)
    : data_directory_(data_directory) {}

bool TerrainService::initialize() {
    const std::filesystem::path root(data_directory_);
    if (!std::filesystem::exists(root)) {
        LOG_ERROR("TRN: Terrain data directory does not exist: {}", data_directory_);
        return false;
    }

    int hgt_count = 0;
    for (const auto& entry : std::filesystem::directory_iterator(root)) {
        if (entry.is_regular_file() && has_hgt_ext_ci(entry.path())) {
            ++hgt_count;
        }
    }

    LOG_INFO("TRN: TerrainService initialized with {} HGT files in {}", hgt_count, data_directory_);
    return true;
}

std::string TerrainService::getTileName(double lat, double lon) const {
    // SRTM tiles are 1°x1°, named by SW corner
    const int lat_int = static_cast<int>(std::floor(lat));
    const int lon_int = static_cast<int>(std::floor(lon));

    std::ostringstream ss;
    ss << (lat_int >= 0 ? 'N' : 'S')
       << std::setw(2) << std::setfill('0') << std::abs(lat_int)
       << (lon_int >= 0 ? 'E' : 'W')
       << std::setw(3) << std::setfill('0') << std::abs(lon_int)
       << ".hgt";
    return ss.str();
}

bool TerrainService::loadTileIfNeeded(double lat, double lon) {
    const std::string tile_name = getTileName(lat, lon);

    {
        std::lock_guard<std::mutex> lock(tiles_mutex_);
        if (tiles_.find(tile_name) != tiles_.end()) {
            return true;
        }
    }

    const std::filesystem::path filepath = std::filesystem::path(data_directory_) / tile_name;
    if (!std::filesystem::exists(filepath)) {
        LOG_WARN("TRN: Tile not found: {}", filepath.string());
        return false;
    }

    auto reader = std::make_shared<SRTMReader>();
    if (!reader->loadTile(filepath.string())) {
        LOG_ERROR("TRN: Failed to load tile: {}", filepath.string());
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(tiles_mutex_);
        tiles_.emplace(tile_name, std::move(reader));
    }
    return true;
}

std::shared_ptr<SRTMReader> TerrainService::getTileForCoordinate(double lat, double lon) {
    const std::string tile_name = getTileName(lat, lon);
    std::lock_guard<std::mutex> lock(tiles_mutex_);
    auto it = tiles_.find(tile_name);
    return (it != tiles_.end()) ? it->second : nullptr;
}

double TerrainService::getElevation(double lat, double lon) {
    if (!loadTileIfNeeded(lat, lon)) {
        return -32768.0;
    }
    auto tile = getTileForCoordinate(lat, lon);
    return tile ? tile->getElevation(lat, lon) : -32768.0;
}

double TerrainService::getElevationBilinear(double lat, double lon) {
    if (!loadTileIfNeeded(lat, lon)) {
        return -32768.0;
    }
    auto tile = getTileForCoordinate(lat, lon);
    return tile ? tile->getElevationBilinear(lat, lon) : -32768.0;
}

Eigen::Vector2d TerrainService::calculateGradient(double lat, double lon) {
    // Central differences with graceful one-sided fallback; returns [dZ/dE, dZ/dN] in m/m.
    const double h = kGradientStep;

    const double zc = getElevationBilinear(lat, lon);
    if (zc == -32768.0) return Eigen::Vector2d::Zero();

    const double zn = getElevationBilinear(lat + h, lon);
    const double zs = getElevationBilinear(lat - h, lon);
    const double ze = getElevationBilinear(lat, lon + h);
    const double zw = getElevationBilinear(lat, lon - h);

    auto diff = [](double zp, double zm, double z0, double hdeg) -> double {
        if (zp != -32768.0 && zm != -32768.0) return (zp - zm) / (2.0 * hdeg);
        if (zp != -32768.0)                    return (zp - z0) / hdeg;
        if (zm != -32768.0)                    return (z0 - zm) / hdeg;
        return 0.0;
    };

    double dlat = diff(zn, zs, zc, h); // meters per degree latitude
    double dlon = diff(ze, zw, zc, h); // meters per degree longitude

    // Convert to meters per meter
    constexpr double meters_per_deg_lat = 111111.0;
    const double meters_per_deg_lon = 111111.0 * std::cos(lat * M_PI / 180.0);

    const double dZ_dN = dlat / meters_per_deg_lat;
    const double dZ_dE = (meters_per_deg_lon > 1e-9) ? (dlon / meters_per_deg_lon) : 0.0;

    return Eigen::Vector2d(dZ_dE, dZ_dN); // [east, north]
}

Eigen::Vector3d TerrainService::getTerrainNormal(double lat, double lon) {
    const Eigen::Vector2d g = calculateGradient(lat, lon); // [dZ/dE, dZ/dN]
    Eigen::Vector3d n(-g.x(), -g.y(), 1.0); // normal to z=f(x,y)
    return n.normalized();
}

double TerrainService::getTerrainSlope(double lat, double lon) {
    const Eigen::Vector2d g = calculateGradient(lat, lon);
    const double slope_rad = std::atan(g.norm()); // tan(theta) = |grad|
    return slope_rad * 180.0 / M_PI;
}

bool TerrainService::preloadRegion(double min_lat, double max_lat,
                                   double min_lon, double max_lon) {
    bool all_loaded = true;
    const int lat_min = static_cast<int>(std::floor(std::min(min_lat, max_lat)));
    const int lat_max = static_cast<int>(std::floor(std::max(min_lat, max_lat)));
    const int lon_min = static_cast<int>(std::floor(std::min(min_lon, max_lon)));
    const int lon_max = static_cast<int>(std::floor(std::max(min_lon, max_lon)));

    for (int lat_i = lat_min; lat_i <= lat_max; ++lat_i) {
        for (int lon_i = lon_min; lon_i <= lon_max; ++lon_i) {
            const double lat_c = lat_i + 0.5;
            const double lon_c = lon_i + 0.5;
            if (!loadTileIfNeeded(lat_c, lon_c)) {
                all_loaded = false;
            }
        }
    }
    return all_loaded;
}

void TerrainService::clearCache() {
    std::lock_guard<std::mutex> lock(tiles_mutex_);
    tiles_.clear();
}

size_t TerrainService::getLoadedTileCount() const {
    std::lock_guard<std::mutex> lock(tiles_mutex_);
    return tiles_.size();
}

} // namespace terrain
