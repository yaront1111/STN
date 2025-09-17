#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "srtm_provider.h"

/**
 * TERRAIN CORRELATION MODULE
 *
 * Provides two-factor authentication for position by correlating
 * measured terrain elevation profiles with predicted profiles from SRTM data.
 *
 * This eliminates false positives from gravity-only matching by requiring
 * both gravity AND terrain signatures to match.
 */
class TerrainCorrelator {
public:
    struct Config {
        double correlation_threshold;      // Minimum terrain correlation for valid match
        double elevation_noise_m;          // Expected radar altimeter noise
        int min_profile_points;            // Minimum points for correlation

        Config() :
            correlation_threshold(0.7),    // 70% terrain correlation for validation
            elevation_noise_m(5.0),         // 5m radar altimeter noise
            min_profile_points(10) {}       // Need at least 10 points
    };

    struct TerrainProfile {
        std::vector<double> elevations;     // Terrain elevations in meters
        std::vector<double> timestamps;     // Time stamps for each elevation
        std::vector<Eigen::Vector3d> positions; // ECEF positions
    };

    struct ValidationResult {
        bool valid;
        double correlation;
        double elevation_rmse;
        std::string reason;
    };

    TerrainCorrelator(const Config& cfg = Config());

    /**
     * Load SRTM elevation data
     */
    bool loadSRTM(const std::string& srtm_path);

    /**
     * Record measured elevation from radar altimeter
     */
    void addMeasurement(double timestamp, const Eigen::Vector3d& position_ECEF,
                       double radar_altitude_m);

    /**
     * Validate a candidate position by comparing terrain profiles
     */
    ValidationResult validateCandidate(const std::vector<Eigen::Vector3d>& candidate_path) const;

    /**
     * Extract terrain profile along a path
     */
    TerrainProfile extractProfile(const std::vector<Eigen::Vector3d>& path) const;

    /**
     * Compute correlation between measured and predicted profiles
     */
    double correlateProfiles(const TerrainProfile& measured,
                           const TerrainProfile& predicted) const;

    /**
     * Clear measurement buffer
     */
    void reset();

    /**
     * Get current profile length
     */
    size_t getProfileLength() const { return measured_profile_.elevations.size(); }

private:
    Config cfg_;
    SRTMProvider srtm_;
    TerrainProfile measured_profile_;

    /**
     * Normalize elevation profile for correlation
     */
    std::vector<double> normalizeProfile(const std::vector<double>& profile) const;
};