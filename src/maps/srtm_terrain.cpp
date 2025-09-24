/**
 * SRTM Terrain Map Implementation
 * Reads and interpolates SRTM elevation data
 */

#include "srtm_terrain.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <iomanip>

namespace Navigation {

SRTMTerrain::SRTMTerrain(const std::string& data_path, const SRTMConfig& config)
    : MapManager("SRTM", data_path), config_(config) {
    
    // Set global bounds (SRTM coverage)
    global_bounds_.lat_min = -56.0;  // SRTM ends at 56°S
    global_bounds_.lat_max = 60.0;   // SRTM ends at 60°N
    global_bounds_.lon_min = -180.0;
    global_bounds_.lon_max = 180.0;
    
    // Resolution in degrees
    resolution_ = config.resolution / 3600.0;  // Arc-seconds to degrees
    
    {
        std::stringstream msg;
        msg << "SRTM terrain map initialized with " << config.resolution
             << " arc-second resolution";
        LOG_INFO(msg.str());
    }
}

bool SRTMTerrain::initialize() {
    LOG_INFO("Initializing SRTM terrain map...");
    
    // Check if data directory exists
    if (config_.data_directory.empty()) {
        LOG_WARN("No SRTM data directory specified");
        config_.data_directory = data_path_ + "/srtm/";
    }
    
    // Test load a tile to verify data availability
    auto test_tile = loadSRTMFile(0, 0);
    if (!test_tile) {
        LOG_WARN("SRTM data not available at test location, continuing anyway");
    }
    
    LOG_INFO("SRTM initialization complete");
    return true;
}

MapQueryResult SRTMTerrain::query(double latitude, double longitude, double altitude) const {
    TerrainQueryResult result = queryTerrain(latitude, longitude);
    
    MapQueryResult base_result;
    base_result.valid = result.valid;
    base_result.value = result.elevation;
    base_result.gradient = result.gradient;
    base_result.confidence = result.confidence;
    
    return base_result;
}

bool SRTMTerrain::isAvailable(double latitude, double longitude) const {
    // Check SRTM coverage
    if (!global_bounds_.contains(latitude, longitude)) {
        return false;
    }
    
    // Check for specific regions without coverage
    if (latitude > 60.0 || latitude < -56.0) {
        return false;
    }
    
    return true;
}

TerrainQueryResult SRTMTerrain::queryTerrain(double lat, double lon) const {
    TerrainQueryResult result;
    result.valid = false;
    
    if (!isAvailable(lat, lon)) {
        {

            std::stringstream msg;

            msg << "Position outside SRTM coverage: " << lat << ", " << lon;

            LOG_DEBUG(msg.str());

        }
        return result;
    }
    
    // Load tile containing this position
    int tile_lat = static_cast<int>(floor(lat));
    int tile_lon = static_cast<int>(floor(lon));
    
    auto base_tile = getTile(tile_lon, tile_lat);
    auto tile = std::static_pointer_cast<SRTMTile>(base_tile);
    if (!tile) {
        tile = loadSRTMFile(tile_lat, tile_lon);
        if (!tile) {
            {

                std::stringstream msg;

                msg << "Failed to load SRTM tile for " << tile_lat << ", " << tile_lon;

                LOG_WARN(msg.str());

            }
            return result;
        }
    }
    
    // Get elevation
    result.elevation = getElevationFromTile(*tile, lat, lon);
    
    // Check for void or water
    if (result.elevation == SRTM_VOID) {
        result.is_void = true;
        result.elevation = 0;  // Default to sea level
    }
    result.is_water = isWater(result.elevation);
    
    // Compute gradient and surface normal
    double x, y;
    geoToPixel(lat, lon, tile_lat, tile_lon, x, y);
    
    result.gradient = computeGradient(*tile, x, y);
    result.surface_normal = computeSurfaceNormal(result.gradient);
    
    // Compute slope and aspect
    result.slope = getSlope(lat, lon);
    result.aspect = getAspect(lat, lon);
    
    result.valid = true;
    result.confidence = result.is_void ? 0.5 : 1.0;
    
    return result;
}

double SRTMTerrain::getElevation(double lat, double lon) const {
    auto result = queryTerrain(lat, lon);
    return result.valid ? result.elevation : 0.0;
}

Vector3d SRTMTerrain::getTerrainGradient(double lat, double lon) const {
    auto result = queryTerrain(lat, lon);
    return result.valid ? result.gradient : Vector3d::Zero();
}

Vector3d SRTMTerrain::getSurfaceNormal(double lat, double lon) const {
    auto result = queryTerrain(lat, lon);
    return result.valid ? result.surface_normal : Vector3d(0, 0, 1);
}

double SRTMTerrain::getSlope(double lat, double lon) const {
    Vector3d gradient = getTerrainGradient(lat, lon);
    double slope_rad = atan(gradient.head(2).norm());
    return slope_rad * 180.0 / M_PI;  // Convert to degrees
}

double SRTMTerrain::getAspect(double lat, double lon) const {
    Vector3d gradient = getTerrainGradient(lat, lon);
    double aspect_rad = atan2(gradient.y(), gradient.x());  // East, North
    double aspect_deg = aspect_rad * 180.0 / M_PI;
    
    // Convert to compass bearing (0=North, 90=East)
    aspect_deg = 90.0 - aspect_deg;
    if (aspect_deg < 0) aspect_deg += 360.0;
    
    return aspect_deg;
}

std::shared_ptr<MapTile> SRTMTerrain::loadTile(int tile_x, int tile_y) const {
    return loadSRTMFile(tile_y, tile_x);  // Note: lat=y, lon=x
}

std::shared_ptr<SRTMTile> SRTMTerrain::loadSRTMFile(double lat, double lon) const {
    int tile_lat = static_cast<int>(floor(lat));
    int tile_lon = static_cast<int>(floor(lon));
    
    std::string filename = getSRTMFilename(tile_lat, tile_lon);
    std::string filepath = config_.data_directory + "/" + filename;
    
    auto tile = std::make_shared<SRTMTile>();
    tile->tile_x = tile_lon;
    tile->tile_y = tile_lat;
    
    // Set bounds
    tile->bounds.lat_min = tile_lat;
    tile->bounds.lat_max = tile_lat + 1;
    tile->bounds.lon_min = tile_lon;
    tile->bounds.lon_max = tile_lon + 1;
    
    // Set resolution
    tile->samples_per_line = (config_.resolution == 1) ? SRTM1_SIZE : SRTM3_SIZE;
    tile->num_lines = tile->samples_per_line;
    tile->resolution = resolution_;
    
    // Try to load file
    if (!parseSRTMData(filepath, *tile)) {
        {

            std::stringstream msg;

            msg << "Failed to load SRTM file: " << filepath;

            LOG_DEBUG(msg.str());

        }
        
        // Create empty tile with zero elevation
        int total_samples = tile->samples_per_line * tile->num_lines;
        tile->elevations.resize(total_samples, 0);
        tile->has_voids = true;
    }
    
    // Fill voids if configured
    if (config_.fill_voids && tile->has_voids) {
        fillVoids(*tile);
    }
    
    // Convert to matrix for base class
    tile->data = MatrixXd(tile->num_lines, tile->samples_per_line);
    for (int i = 0; i < tile->num_lines; ++i) {
        for (int j = 0; j < tile->samples_per_line; ++j) {
            int idx = i * tile->samples_per_line + j;
            tile->data(i, j) = tile->elevations[idx];
        }
    }
    
    return tile;
}

std::string SRTMTerrain::getSRTMFilename(int lat, int lon) const {
    // SRTM filename format: N00E000.hgt or S00W000.hgt
    std::stringstream ss;
    
    // Latitude
    if (lat >= 0) {
        ss << "N" << std::setfill('0') << std::setw(2) << lat;
    } else {
        ss << "S" << std::setfill('0') << std::setw(2) << -lat;
    }
    
    // Longitude
    if (lon >= 0) {
        ss << "E" << std::setfill('0') << std::setw(3) << lon;
    } else {
        ss << "W" << std::setfill('0') << std::setw(3) << -lon;
    }
    
    ss << ".hgt";
    
    return ss.str();
}

bool SRTMTerrain::parseSRTMData(const std::string& filename, SRTMTile& tile) const {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    
    // Get file size
    file.seekg(0, std::ios::end);
    size_t file_size = file.tellg();
    file.seekg(0, std::ios::beg);
    
    // Calculate expected size
    size_t expected_size = tile.samples_per_line * tile.num_lines * sizeof(int16_t);
    if (file_size != expected_size) {
        {

            std::stringstream msg;

            msg << "SRTM file size mismatch: " << file_size << " != " << expected_size;

            LOG_WARN(msg.str());

        }
        return false;
    }
    
    // Read elevation data
    tile.elevations.resize(tile.samples_per_line * tile.num_lines);
    file.read(reinterpret_cast<char*>(tile.elevations.data()), file_size);
    
    // SRTM data is big-endian, swap if needed
    tile.void_count = 0;
    for (auto& elev : tile.elevations) {
        elev = swapEndian(elev);
        
        if (elev == SRTM_VOID) {
            tile.has_voids = true;
            tile.void_count++;
        }
    }
    
    if (tile.void_count > 0) {
        {

            std::stringstream msg;

            msg << "SRTM tile has " << tile.void_count << " void pixels";

            LOG_DEBUG(msg.str());

        }
    }
    
    return true;
}

int16_t SRTMTerrain::swapEndian(int16_t value) const {
    // Swap bytes for big-endian to little-endian conversion
    return ((value & 0xFF) << 8) | ((value >> 8) & 0xFF);
}

double SRTMTerrain::getElevationFromTile(const SRTMTile& tile, double lat, double lon) const {
    double x, y;
    geoToPixel(lat, lon, tile.tile_y, tile.tile_x, x, y);
    
    if (config_.interpolate) {
        return interpolateElevation(tile, x, y);
    } else {
        // Nearest neighbor
        int ix = static_cast<int>(round(x));
        int iy = static_cast<int>(round(y));
        
        // Bounds check
        ix = std::max(0, std::min(tile.samples_per_line - 1, ix));
        iy = std::max(0, std::min(tile.num_lines - 1, iy));
        
        int idx = iy * tile.samples_per_line + ix;
        return tile.elevations[idx];
    }
}

double SRTMTerrain::interpolateElevation(const SRTMTile& tile, double x, double y) const {
    // Bilinear interpolation
    int x0 = static_cast<int>(floor(x));
    int y0 = static_cast<int>(floor(y));
    int x1 = x0 + 1;
    int y1 = y0 + 1;
    
    // Bounds check
    x0 = std::max(0, std::min(tile.samples_per_line - 1, x0));
    x1 = std::max(0, std::min(tile.samples_per_line - 1, x1));
    y0 = std::max(0, std::min(tile.num_lines - 1, y0));
    y1 = std::max(0, std::min(tile.num_lines - 1, y1));
    
    // Get elevations at corners
    double e00 = tile.elevations[y0 * tile.samples_per_line + x0];
    double e10 = tile.elevations[y0 * tile.samples_per_line + x1];
    double e01 = tile.elevations[y1 * tile.samples_per_line + x0];
    double e11 = tile.elevations[y1 * tile.samples_per_line + x1];
    
    // Handle voids
    if (e00 == SRTM_VOID) e00 = 0;
    if (e10 == SRTM_VOID) e10 = 0;
    if (e01 == SRTM_VOID) e01 = 0;
    if (e11 == SRTM_VOID) e11 = 0;
    
    // Fractional parts
    double fx = x - x0;
    double fy = y - y0;
    
    // Bilinear interpolation
    double e0 = e00 * (1 - fx) + e10 * fx;
    double e1 = e01 * (1 - fx) + e11 * fx;
    
    return e0 * (1 - fy) + e1 * fy;
}

void SRTMTerrain::fillVoids(SRTMTile& tile) const {
    if (!tile.has_voids) return;
    
    std::vector<int16_t> filled = tile.elevations;
    int radius = static_cast<int>(config_.void_fill_radius);
    
    for (int y = 0; y < tile.num_lines; ++y) {
        for (int x = 0; x < tile.samples_per_line; ++x) {
            int idx = y * tile.samples_per_line + x;
            
            if (tile.elevations[idx] == SRTM_VOID) {
                filled[idx] = static_cast<int16_t>(
                    averageNearbyElevations(tile, x, y, radius)
                );
            }
        }
    }
    
    tile.elevations = filled;
    tile.has_voids = false;
    tile.void_count = 0;
}

double SRTMTerrain::averageNearbyElevations(const SRTMTile& tile, int cx, int cy, int radius) const {
    double sum = 0;
    int count = 0;
    
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            int x = cx + dx;
            int y = cy + dy;
            
            // Bounds check
            if (x < 0 || x >= tile.samples_per_line || 
                y < 0 || y >= tile.num_lines) {
                continue;
            }
            
            int idx = y * tile.samples_per_line + x;
            if (tile.elevations[idx] != SRTM_VOID) {
                sum += tile.elevations[idx];
                count++;
            }
        }
    }
    
    return (count > 0) ? (sum / count) : 0.0;
}

Vector3d SRTMTerrain::computeGradient(const SRTMTile& tile, double x, double y) const {
    Vector3d gradient;
    
    int ix = static_cast<int>(x);
    int iy = static_cast<int>(y);
    
    // Bounds check
    if (ix <= 0 || ix >= tile.samples_per_line - 1 ||
        iy <= 0 || iy >= tile.num_lines - 1) {
        return Vector3d::Zero();
    }
    
    // Get neighboring elevations
    double e_west = interpolateElevation(tile, x - 1, y);
    double e_east = interpolateElevation(tile, x + 1, y);
    double e_south = interpolateElevation(tile, x, y + 1);
    double e_north = interpolateElevation(tile, x, y - 1);
    
    // Compute gradient (meters per pixel)
    double pixel_size_m = resolution_ * 111111.0;  // Approximate meters per degree
    
    gradient.x() = (e_north - e_south) / (2 * pixel_size_m);  // North
    gradient.y() = (e_east - e_west) / (2 * pixel_size_m);    // East
    gradient.z() = 0;  // Vertical component not used for terrain
    
    return gradient;
}

Vector3d SRTMTerrain::computeSurfaceNormal(const Vector3d& gradient) const {
    // Surface normal from gradient
    Vector3d normal(-gradient.x(), -gradient.y(), 1.0);
    normal.normalize();
    return normal;
}

void SRTMTerrain::pixelToGeo(int x, int y, int tile_lat, int tile_lon, 
                            double& lat, double& lon) const {
    // Convert pixel coordinates to geographic
    int samples = (config_.resolution == 1) ? SRTM1_SIZE : SRTM3_SIZE;
    
    // SRTM pixels start from top-left (NW corner)
    lat = tile_lat + 1.0 - (y / static_cast<double>(samples - 1));
    lon = tile_lon + (x / static_cast<double>(samples - 1));
}

void SRTMTerrain::geoToPixel(double lat, double lon, int tile_lat, int tile_lon,
                            double& x, double& y) const {
    // Convert geographic to pixel coordinates
    int samples = (config_.resolution == 1) ? SRTM1_SIZE : SRTM3_SIZE;
    
    // Fractional position within tile
    double lat_frac = lat - tile_lat;
    double lon_frac = lon - tile_lon;
    
    // Convert to pixel coordinates
    x = lon_frac * (samples - 1);
    y = (1.0 - lat_frac) * (samples - 1);
}

bool SRTMTerrain::isWater(double elevation) const {
    return elevation <= config_.water_level;
}

} // namespace Navigation