#include "srtm_provider.h"
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>

const int SRTMProvider::SRTM3_SIZE;

SRTMProvider::SRTMProvider() : data_loaded_(false), use_synthetic_(false) {
}

SRTMProvider::~SRTMProvider() {
}

bool SRTMProvider::loadData(const std::string& srtm_directory) {
    data_directory_ = srtm_directory;

    // Try to load tiles for Switzerland region (around lat 47°, lon 8°)
    // In production, this would load tiles on demand based on area of operation
    bool any_loaded = false;

    // Load a grid of tiles around Switzerland
    for (int lat = 45; lat <= 48; ++lat) {
        for (int lon = 6; lon <= 10; ++lon) {
            if (loadTile(lat, lon)) {
                any_loaded = true;
                std::cout << "✓ Loaded SRTM tile N" << lat << "E" << std::setfill('0')
                         << std::setw(3) << lon << "\n";
            }
        }
    }

    if (any_loaded) {
        data_loaded_ = true;
        use_synthetic_ = false;
        std::cout << "✓ SRTM terrain data loaded successfully\n";
    } else {
        // Fallback to synthetic terrain
        data_loaded_ = true;
        use_synthetic_ = true;
        std::cout << "WARNING: No SRTM tiles found, using synthetic terrain\n";
        std::cout << "  (Download SRTM .hgt files to: " << srtm_directory << ")\n";
    }

    return true;
}

bool SRTMProvider::loadTile(int lat_deg, int lon_deg) {
    std::string filename = getTileFilename(lat_deg, lon_deg);
    std::string filepath = data_directory_ + "/" + filename;

    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        // Try alternate naming convention
        filepath = data_directory_ + "/SRTM3/" + filename;
        file.open(filepath, std::ios::binary);
        if (!file.is_open()) {
            return false;
        }
    }

    // SRTM3 files are exactly 1201*1201*2 bytes (big-endian signed 16-bit integers)
    const size_t expected_size = SRTM3_SIZE * SRTM3_SIZE * 2;

    // Get file size
    file.seekg(0, std::ios::end);
    size_t file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    if (file_size != expected_size) {
        std::cerr << "WARNING: SRTM file " << filepath << " has incorrect size\n";
        std::cerr << "  Expected: " << expected_size << " bytes, got: " << file_size << "\n";
        return false;
    }

    // Read elevation data
    std::vector<int16_t> tile_data(SRTM3_SIZE * SRTM3_SIZE);
    file.read(reinterpret_cast<char*>(tile_data.data()), expected_size);

    // Convert from big-endian to native byte order
    for (auto& val : tile_data) {
        val = (val >> 8) | ((val & 0xFF) << 8);
        // SRTM uses -32768 for void/water - replace with 0
        if (val == -32768) val = 0;
    }

    // Store in cache
    elevation_tiles_[std::make_pair(lat_deg, lon_deg)] = tile_data;

    return true;
}

std::string SRTMProvider::getTileFilename(int lat_deg, int lon_deg) const {
    std::stringstream ss;

    // SRTM naming convention: N47E008.hgt
    if (lat_deg >= 0) {
        ss << "N" << std::setfill('0') << std::setw(2) << lat_deg;
    } else {
        ss << "S" << std::setfill('0') << std::setw(2) << (-lat_deg);
    }

    if (lon_deg >= 0) {
        ss << "E" << std::setfill('0') << std::setw(3) << lon_deg;
    } else {
        ss << "W" << std::setfill('0') << std::setw(3) << (-lon_deg);
    }

    ss << ".hgt";
    return ss.str();
}

double SRTMProvider::getElevation(double lat_rad, double lon_rad) const {
    double lat_deg = lat_rad * 180.0 / M_PI;
    double lon_deg = lon_rad * 180.0 / M_PI;

    // Find which tile this point belongs to
    int tile_lat = static_cast<int>(std::floor(lat_deg));
    int tile_lon = static_cast<int>(std::floor(lon_deg));

    // Check if we have this tile
    auto tile_it = elevation_tiles_.find(std::make_pair(tile_lat, tile_lon));
    if (tile_it == elevation_tiles_.end()) {
        // No real data - use synthetic
        if (use_synthetic_ || !data_loaded_) {
            return generateSyntheticTerrain(lat_rad, lon_rad);
        }
        // No data for this region
        return 0.0;
    }

    const auto& tile_data = tile_it->second;

    // Calculate position within tile (0 to 1)
    double lat_frac = lat_deg - tile_lat;
    double lon_frac = lon_deg - tile_lon;

    // SRTM data is stored from north to south, west to east
    // So row 0 is the northernmost, row 1200 is the southernmost
    int lat_idx = static_cast<int>((1.0 - lat_frac) * (SRTM3_SIZE - 1));
    int lon_idx = static_cast<int>(lon_frac * (SRTM3_SIZE - 1));

    // Clamp to valid range
    lat_idx = std::max(0, std::min(SRTM3_SIZE - 2, lat_idx));
    lon_idx = std::max(0, std::min(SRTM3_SIZE - 2, lon_idx));

    // Get fractional part for interpolation
    double lat_interp = (1.0 - lat_frac) * (SRTM3_SIZE - 1) - lat_idx;
    double lon_interp = lon_frac * (SRTM3_SIZE - 1) - lon_idx;

    // Bilinear interpolation
    return interpolateElevation(lat_idx, lon_idx, lat_interp, lon_interp, tile_data);
}

double SRTMProvider::interpolateElevation(int lat_idx, int lon_idx,
                                         double lat_frac, double lon_frac,
                                         const std::vector<int16_t>& tile_data) const {
    // Get four corner points
    double h00 = tile_data[lat_idx * SRTM3_SIZE + lon_idx];
    double h01 = tile_data[lat_idx * SRTM3_SIZE + lon_idx + 1];
    double h10 = tile_data[(lat_idx + 1) * SRTM3_SIZE + lon_idx];
    double h11 = tile_data[(lat_idx + 1) * SRTM3_SIZE + lon_idx + 1];

    // Bilinear interpolation
    double h0 = h00 * (1 - lon_frac) + h01 * lon_frac;
    double h1 = h10 * (1 - lon_frac) + h11 * lon_frac;
    double h = h0 * (1 - lat_frac) + h1 * lat_frac;

    return h;
}

double SRTMProvider::generateSyntheticTerrain(double lat_rad, double lon_rad) const {
    // Fallback synthetic terrain for areas without SRTM data
    double lat_deg = lat_rad * 180.0 / M_PI;
    double lon_deg = lon_rad * 180.0 / M_PI;

    // Base elevation
    double base_elevation = 500.0;

    // Large-scale mountains
    double mountains = 800.0 * std::sin(lat_deg * 0.1) * std::cos(lon_deg * 0.1);

    // Medium-scale hills
    double hills = 200.0 * std::sin(lat_deg * 0.5 + 1.0) * std::cos(lon_deg * 0.5 + 2.0);

    // Small-scale variation
    double variation = 50.0 * std::sin(lat_deg * 2.0 + 3.0) * std::cos(lon_deg * 2.0 + 4.0);

    // Alpine ridge for Switzerland region
    if (lat_deg > 45 && lat_deg < 48 && lon_deg > 6 && lon_deg < 10) {
        double alpine = 1500.0 * std::exp(-std::pow((lat_deg - 46.5), 2) / 2.0);
        return base_elevation + mountains + hills + variation + alpine;
    }

    return base_elevation + mountains + hills + variation;
}

Eigen::Vector3d SRTMProvider::ecefToLla(const Eigen::Vector3d& ecef) const {
    double lat, lon, alt;
    ecefToGeodetic(ecef, lat, lon, alt);
    return Eigen::Vector3d(lat, lon, alt);
}

void SRTMProvider::ecefToGeodetic(const Eigen::Vector3d& ecef,
                                  double& lat_rad, double& lon_rad, double& alt_m) const {
    // WGS84 ellipsoid parameters
    const double a = 6378137.0;        // Semi-major axis
    const double e2 = 0.00669437999014; // First eccentricity squared

    double x = ecef(0);
    double y = ecef(1);
    double z = ecef(2);

    // Calculate longitude
    lon_rad = std::atan2(y, x);

    // Calculate latitude and altitude using iterative method
    double p = std::sqrt(x*x + y*y);
    double lat_init = std::atan2(z, p * (1.0 - e2));

    for (int i = 0; i < 5; ++i) {
        double N = a / std::sqrt(1.0 - e2 * std::sin(lat_init) * std::sin(lat_init));
        double h = p / std::cos(lat_init) - N;
        lat_init = std::atan2(z, p * (1.0 - e2 * N / (N + h)));
    }

    lat_rad = lat_init;
    double N = a / std::sqrt(1.0 - e2 * std::sin(lat_rad) * std::sin(lat_rad));
    alt_m = p / std::cos(lat_rad) - N;
}

bool SRTMProvider::hasDataForRegion(double lat_rad, double lon_rad) const {
    if (use_synthetic_) return true;  // Always have synthetic data

    double lat_deg = lat_rad * 180.0 / M_PI;
    double lon_deg = lon_rad * 180.0 / M_PI;

    int tile_lat = static_cast<int>(std::floor(lat_deg));
    int tile_lon = static_cast<int>(std::floor(lon_deg));

    return elevation_tiles_.find(std::make_pair(tile_lat, tile_lon)) != elevation_tiles_.end();
}

std::vector<double> SRTMProvider::getElevationProfile(const std::vector<Eigen::Vector3d>& path_ecef) const {
    std::vector<double> profile;

    for (const auto& pos : path_ecef) {
        auto lla = ecefToLla(pos);
        double elev = getElevation(lla(0), lla(1));
        profile.push_back(elev);
    }

    return profile;
}