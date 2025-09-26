#include "terrain/srtm_reader.h"
#include "core/utils/logging.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

namespace terrain {

SRTMReader::SRTMReader() = default;

static inline int16_t be_to_i16(unsigned char hi, unsigned char lo) {
    // Assemble big-endian signed 16-bit
    uint16_t u = static_cast<uint16_t>((hi << 8) | lo);
    return static_cast<int16_t>(u);
}

bool SRTMReader::parseTileName(const std::string& filename,
                               double& sw_lat_deg,
                               double& sw_lon_deg) {
    // Expect stem like N32E034.hgt / s31w117.hgt
    const auto stem = std::filesystem::path(filename).stem().string();
    if (stem.size() < 7) return false;

    const char NS = static_cast<char>(std::toupper(stem[0]));
    const char EW = static_cast<char>(std::toupper(stem[3]));
    if ((NS!='N' && NS!='S') || (EW!='E' && EW!='W')) return false;

    // Strict positions: N[01..90][E/W][000..180]
    // Lat: two digits at [1..2]
    // Lon: three digits at [4..6]
    int lat = 0, lon = 0;
    try {
        lat = std::stoi(stem.substr(1, 2));
        lon = std::stoi(stem.substr(4, 3));
    } catch (...) {
        return false;
    }
    if (lat < 0 || lat > 90 || lon < 0 || lon > 180) return false;

    sw_lat_deg = (NS == 'N') ? +lat : -lat;
    sw_lon_deg = (EW == 'E') ? +lon : -lon;
    return true;
}

bool SRTMReader::loadTile(const std::string& filepath) {
    loaded_ = false;
    elevation_data_.clear();
    size_ = 0;

    // Parse SW corner from name
    double sw_lat = 0.0, sw_lon = 0.0;
    if (!parseTileName(std::filesystem::path(filepath).filename().string(), sw_lat, sw_lon)) {
        LOG_ERROR("TRN: Failed to parse tile name: {}", filepath);
        return false;
    }

    std::ifstream f(filepath, std::ios::binary);
    if (!f) {
        LOG_ERROR("TRN: Failed to open HGT file: {}", filepath);
        return false;
    }

    f.seekg(0, std::ios::end);
    const std::streamoff bytes = f.tellg();
    f.seekg(0, std::ios::beg);

    const std::streamoff bytes_srtm3  = static_cast<std::streamoff>(1201) * 1201 * 2;
    const std::streamoff bytes_srtm1  = static_cast<std::streamoff>(3601) * 3601 * 2;

    if (bytes == bytes_srtm3) {
        size_ = 1201;
    } else if (bytes == bytes_srtm1) {
        size_ = 3601;
    } else {
        LOG_ERROR("TRN: Unknown HGT file size: {} ({} expected or {})",
                  static_cast<long long>(bytes),
                  static_cast<long long>(bytes_srtm3),
                  static_cast<long long>(bytes_srtm1));
        return false;
    }

    elevation_data_.resize(static_cast<size_t>(size_) * size_);

    // Read as raw bytes and assemble big-endian samples safely
    for (int i = 0; i < size_ * size_; ++i) {
        unsigned char b[2];
        f.read(reinterpret_cast<char*>(b), 2);
        if (!f) {
            LOG_ERROR("TRN: Error reading HGT data at sample {}", i);
            elevation_data_.clear();
            size_ = 0;
            return false;
        }
        elevation_data_[i] = be_to_i16(b[0], b[1]);
    }

    min_lat_ = sw_lat;
    min_lon_ = sw_lon;
    max_lat_ = sw_lat + 1.0;
    max_lon_ = sw_lon + 1.0;
    tile_name_ = std::filesystem::path(filepath).filename().string();
    loaded_ = true;

    // Log min/max elevation statistics to detect data issues
    int16_t minv = INT16_MAX, maxv = INT16_MIN;
    for (auto h : elevation_data_) {
        if (h == -32768) continue; // skip voids
        if (h < minv) minv = h;
        if (h > maxv) maxv = h;
    }

    LOG_INFO("TRN: Loaded SRTM tile {} ({}x{}) {:.1f}–{:.1f}°lat, {:.1f}–{:.1f}°lon",
             tile_name_, size_, size_, min_lat_, max_lat_, min_lon_, max_lon_);
    LOG_INFO("TRN: Tile {} elevation stats: min={} m, max={} m", tile_name_, (int)minv, (int)maxv);
    return true;
}

bool SRTMReader::containsCoordinate(double lat, double lon) const {
    if (!loaded_) return false;
    // SRTM includes both edges; allow inclusive bounds
    return (lat >= min_lat_ && lat <= max_lat_ && lon >= min_lon_ && lon <= max_lon_);
}

void SRTMReader::getPixelIndices(double lat, double lon, int& row, int& col) const {
    // Map to discrete pixel (nearest)
    const double lat_norm = (max_lat_ - lat) / (max_lat_ - min_lat_); // 0 at north edge
    const double lon_norm = (lon - min_lon_) / (max_lon_ - min_lon_); // 0 at west edge
    row = std::clamp(static_cast<int>(std::round(lat_norm * (size_ - 1))), 0, size_ - 1);
    col = std::clamp(static_cast<int>(std::round(lon_norm * (size_ - 1))), 0, size_ - 1);
}

void SRTMReader::getPixelIndicesFrac(double lat, double lon, double& r, double& c) const {
    const double lat_norm = (max_lat_ - lat) / (max_lat_ - min_lat_);
    const double lon_norm = (lon - min_lon_) / (max_lon_ - min_lon_);
    r = std::clamp(lat_norm * (size_ - 1), 0.0, static_cast<double>(size_ - 1));
    c = std::clamp(lon_norm * (size_ - 1), 0.0, static_cast<double>(size_ - 1));
}

double SRTMReader::getElevation(double lat, double lon) const {
    if (!loaded_ || !containsCoordinate(lat, lon)) {
        return -32768.0;
    }
    int r, c;
    getPixelIndices(lat, lon, r, c);
    const int idx = r * size_ + c;
    const int16_t h = elevation_data_[idx];
    return isVoid(h) ? -32768.0 : static_cast<double>(h);
}

double SRTMReader::getElevationBilinear(double lat, double lon) const {
    if (!loaded_ || !containsCoordinate(lat, lon)) {
        return -32768.0;
    }

    double rf, cf;
    getPixelIndicesFrac(lat, lon, rf, cf);

    const int r0 = static_cast<int>(std::floor(rf));
    const int c0 = static_cast<int>(std::floor(cf));
    const int r1 = std::min(r0 + 1, size_ - 1);
    const int c1 = std::min(c0 + 1, size_ - 1);

    const double tr = rf - r0;
    const double tc = cf - c0;

    auto H = [&](int r, int c) -> double {
        const int16_t h = elevation_data_[r * size_ + c];
        return isVoid(h) ? std::numeric_limits<double>::quiet_NaN()
                         : static_cast<double>(h);
    };

    const double e00 = H(r0, c0);
    const double e01 = H(r0, c1);
    const double e10 = H(r1, c0);
    const double e11 = H(r1, c1);

    // If any are NaN (void), average the valid ones (smoother than nearest-only)
    int valid = 0;
    double sum = 0.0;
    for (double e : {e00, e01, e10, e11}) {
        if (std::isfinite(e)) { sum += e; ++valid; }
    }
    if (valid == 0) return -32768.0;
    if (valid < 4)  return sum / valid;

    // Standard bilinear
    const double e0 = e00 * (1.0 - tc) + e01 * tc;
    const double e1 = e10 * (1.0 - tc) + e11 * tc;
    return e0 * (1.0 - tr) + e1 * tr;
}

} // namespace terrain
