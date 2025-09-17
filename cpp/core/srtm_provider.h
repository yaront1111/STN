#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <map>

/**
 * SRTM (Shuttle Radar Topography Mission) ELEVATION DATA PROVIDER
 *
 * Provides access to global terrain elevation data from SRTM
 * Used for terrain correlation in two-factor position authentication
 */
class SRTMProvider {
public:
    SRTMProvider();
    ~SRTMProvider();

    /**
     * Load SRTM data files from directory
     */
    bool loadData(const std::string& srtm_directory);

    /**
     * Get terrain elevation at a specific location
     * @param lat_rad Latitude in radians
     * @param lon_rad Longitude in radians
     * @return Elevation in meters above sea level
     */
    double getElevation(double lat_rad, double lon_rad) const;

    /**
     * Convert ECEF to geodetic coordinates
     */
    Eigen::Vector3d ecefToLla(const Eigen::Vector3d& ecef) const;

    /**
     * Check if elevation data is available for a region
     */
    bool hasDataForRegion(double lat_rad, double lon_rad) const;

    /**
     * Get elevation profile along a path
     */
    std::vector<double> getElevationProfile(const std::vector<Eigen::Vector3d>& path_ecef) const;

private:
    /**
     * Load a single SRTM .hgt tile
     */
    bool loadTile(int lat_deg, int lon_deg);

    /**
     * Get tile filename for given coordinates
     */
    std::string getTileFilename(int lat_deg, int lon_deg) const;

    // NO SYNTHETIC TERRAIN - REAL DATA ONLY

    /**
     * Simple ECEF to geodetic conversion
     */
    void ecefToGeodetic(const Eigen::Vector3d& ecef,
                        double& lat_rad, double& lon_rad, double& alt_m) const;

    /**
     * Bilinear interpolation of elevation data
     */
    double interpolateElevation(int lat_idx, int lon_idx,
                               double lat_frac, double lon_frac,
                               const std::vector<int16_t>& tile_data) const;

    bool data_loaded_;
    // NO SYNTHETIC DATA - PRODUCTION ONLY
    std::string data_directory_;

    // SRTM3 has 1201x1201 points per 1-degree tile (3 arc-second resolution)
    static const int SRTM3_SIZE = 1201;

    // Cache of loaded tiles: key is (lat, lon) in degrees
    std::map<std::pair<int, int>, std::vector<int16_t>> elevation_tiles_;
};