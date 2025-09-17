#include "terrain_correlator.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>

TerrainCorrelator::TerrainCorrelator(const Config& cfg) : cfg_(cfg) {
}

bool TerrainCorrelator::loadSRTM(const std::string& srtm_path) {
    bool loaded = srtm_.loadData(srtm_path);
    if (loaded) {
        std::cout << "✓ Loaded SRTM terrain data for correlation\n";
    } else {
        std::cerr << "ERROR: Failed to load SRTM data from " << srtm_path << "\n";
    }
    return loaded;
}

void TerrainCorrelator::addMeasurement(double timestamp,
                                      const Eigen::Vector3d& position_ECEF,
                                      double radar_altitude_m) {
    // Calculate MEASURED terrain elevation from aircraft position and radar altitude
    auto lla = srtm_.ecefToLla(position_ECEF);
    double aircraft_altitude = lla(2);  // Aircraft altitude above sea level
    double measured_terrain_elev = aircraft_altitude - radar_altitude_m;  // This is what we actually measured!

    // Store the measurement
    measured_profile_.timestamps.push_back(timestamp);
    measured_profile_.positions.push_back(position_ECEF);
    measured_profile_.elevations.push_back(measured_terrain_elev);

    // Keep buffer size limited
    while (measured_profile_.elevations.size() > 200) {
        measured_profile_.timestamps.erase(measured_profile_.timestamps.begin());
        measured_profile_.positions.erase(measured_profile_.positions.begin());
        measured_profile_.elevations.erase(measured_profile_.elevations.begin());
    }
}

TerrainCorrelator::ValidationResult TerrainCorrelator::validateCandidate(
    const std::vector<Eigen::Vector3d>& candidate_path) const {

    ValidationResult result;
    result.valid = false;
    result.correlation = 0.0;  // Initialize!
    result.elevation_rmse = 0.0;

    // Need enough measurements
    if (measured_profile_.elevations.size() < cfg_.min_profile_points) {
        result.reason = "Insufficient profile points";
        return result;
    }

    // Extract predicted terrain profile along candidate path
    // Match the sizes by using the minimum of both
    std::vector<Eigen::Vector3d> matched_path;
    size_t measured_size = measured_profile_.elevations.size();
    size_t candidate_size = candidate_path.size();

    // Use the smaller size to ensure we can correlate
    size_t use_size = std::min(measured_size, candidate_size);

    if (use_size < cfg_.min_profile_points) {
        result.reason = "Not enough overlapping points";
        return result;
    }

    // Take last N points from both profiles
    matched_path.assign(candidate_path.end() - use_size, candidate_path.end());

    // Also trim the measured profile to match
    TerrainProfile measured_trimmed;
    measured_trimmed.elevations.assign(
        measured_profile_.elevations.end() - use_size,
        measured_profile_.elevations.end());
    measured_trimmed.positions.assign(
        measured_profile_.positions.end() - use_size,
        measured_profile_.positions.end());

    TerrainProfile predicted = extractProfile(matched_path);

    if (predicted.elevations.size() != measured_trimmed.elevations.size()) {
        result.reason = "Profile size still mismatched after adjustment";
        return result;
    }

    // Compute correlation with trimmed profiles
    result.correlation = correlateProfiles(measured_trimmed, predicted);

    // Compute RMSE
    double sum_sq_error = 0;
    for (size_t i = 0; i < measured_trimmed.elevations.size(); ++i) {
        double error = measured_trimmed.elevations[i] - predicted.elevations[i];
        sum_sq_error += error * error;
    }
    result.elevation_rmse = std::sqrt(sum_sq_error / measured_trimmed.elevations.size());

    // Check if valid
    if (result.correlation >= cfg_.correlation_threshold) {
        result.valid = true;
        result.reason = "Terrain profile matches";
    } else {
        result.reason = "Low terrain correlation";
    }

    return result;
}

TerrainCorrelator::TerrainProfile TerrainCorrelator::extractProfile(
    const std::vector<Eigen::Vector3d>& path) const {

    TerrainProfile profile;

    for (const auto& pos : path) {
        // Convert to lat/lon
        auto lla = srtm_.ecefToLla(pos);

        // Get terrain elevation from SRTM
        double elev = srtm_.getElevation(lla(0), lla(1));

        profile.positions.push_back(pos);
        profile.elevations.push_back(elev);
    }

    return profile;
}

double TerrainCorrelator::correlateProfiles(const TerrainProfile& measured,
                                           const TerrainProfile& predicted) const {
    if (measured.elevations.size() != predicted.elevations.size() ||
        measured.elevations.empty()) {
        return -1.0;
    }

    // Normalize both profiles
    auto meas_norm = normalizeProfile(measured.elevations);
    auto pred_norm = normalizeProfile(predicted.elevations);

    // Compute correlation coefficient
    size_t n = meas_norm.size();
    double sum_xy = 0, sum_x = 0, sum_y = 0, sum_x2 = 0, sum_y2 = 0;

    for (size_t i = 0; i < n; ++i) {
        sum_xy += meas_norm[i] * pred_norm[i];
        sum_x += meas_norm[i];
        sum_y += pred_norm[i];
        sum_x2 += meas_norm[i] * meas_norm[i];
        sum_y2 += pred_norm[i] * pred_norm[i];
    }

    double numerator = n * sum_xy - sum_x * sum_y;
    double denominator = std::sqrt((n * sum_x2 - sum_x * sum_x) *
                                  (n * sum_y2 - sum_y * sum_y));

    if (denominator == 0) return 0;

    return numerator / denominator;
}

std::vector<double> TerrainCorrelator::normalizeProfile(const std::vector<double>& profile) const {
    if (profile.empty()) return profile;

    // Compute mean and std dev
    double mean = std::accumulate(profile.begin(), profile.end(), 0.0) / profile.size();

    double var = 0;
    for (double val : profile) {
        var += (val - mean) * (val - mean);
    }
    var /= profile.size();
    double stddev = std::sqrt(var);

    // Normalize
    std::vector<double> normalized;
    if (stddev > 1e-3) {  // Avoid division by zero for flat terrain
        for (double val : profile) {
            normalized.push_back((val - mean) / stddev);
        }
    } else {
        // Flat terrain - return zeros
        normalized.resize(profile.size(), 0.0);
    }

    return normalized;
}

void TerrainCorrelator::reset() {
    measured_profile_.timestamps.clear();
    measured_profile_.positions.clear();
    measured_profile_.elevations.clear();
}