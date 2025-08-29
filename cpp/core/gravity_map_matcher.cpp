#include "gravity_map_matcher.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>
#include <chrono>

GravityMapMatcher::GravityMapMatcher(const Config& cfg) : cfg_(cfg) {
    signature_buffer_.clear();
}

void GravityMapMatcher::addMeasurement(const GravityMeasurement& meas) {
    signature_buffer_.push_back(meas);
    
    // Keep buffer at maximum length
    while (signature_buffer_.size() > cfg_.signature_length) {
        signature_buffer_.pop_front();
    }
}

GravityMapMatcher::MatchResult GravityMapMatcher::findMatch(
    const GravityGradientProvider& gravity_model) {
    
    MatchResult result;
    result.valid = false;
    
    // Need minimum measurements
    if (signature_buffer_.size() < cfg_.min_measurements) {
        return result;
    }
    
    // Extract measured signature
    std::vector<double> measured_signature;
    std::vector<Eigen::Vector3d> estimated_path;
    
    for (const auto& meas : signature_buffer_) {
        measured_signature.push_back(meas.anomaly_mgal);
        estimated_path.push_back(meas.position_ECEF);
    }
    
    // Normalize for correlation
    auto measured_norm = normalizeSignature(measured_signature);
    
    // Get search center (current estimated position)
    Eigen::Vector3d search_center = signature_buffer_.back().position_ECEF;
    
    // Generate search grid
    auto search_grid = generateSearchGrid(search_center);
    
    std::cout << "  Search grid size: " << search_grid.size() << " points\n";
    std::cout << "  Search radius: " << cfg_.search_radius_m << "m\n";
    std::cout << "  Grid resolution: " << cfg_.grid_resolution_m << "m\n";
    
    // Track best match
    double best_correlation = -1.0;
    Eigen::Vector3d best_position;
    std::vector<Eigen::Vector3d> best_path;
    
    int points_evaluated = 0;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Search each grid point
    for (const auto& grid_point : search_grid) {
        points_evaluated++;
        // Compute offset from estimated to grid point
        Eigen::Vector3d offset = grid_point - search_center;
        
        // Apply offset to entire path
        std::vector<Eigen::Vector3d> test_path;
        for (const auto& pos : estimated_path) {
            test_path.push_back(pos + offset);
        }
        
        // Extract gravity signature from map at this path
        auto map_signature = extractMapSignature(gravity_model, test_path);
        
        if (map_signature.empty()) continue;
        
        // Normalize and correlate
        auto map_norm = normalizeSignature(map_signature);
        double correlation = computeCorrelation(measured_norm, map_norm);
        
        // Track best match
        if (correlation > best_correlation) {
            best_correlation = correlation;
            best_position = grid_point;
            best_path = test_path;
        }
        
        // Early exit if excellent match found
        if (correlation > 0.99) {
            std::cout << "  Found excellent match early at point " << points_evaluated << "\n";
            break;
        }
        
        // Progress report every 100 points
        if (points_evaluated % 100 == 0) {
            std::cout << "  Evaluated " << points_evaluated << "/" << search_grid.size() 
                      << " points, best correlation: " << best_correlation << "\n";
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "  Search completed in " << duration.count() << "ms\n";
    std::cout << "  Points evaluated: " << points_evaluated << "/" << search_grid.size() << "\n";
    
    // Check if match is good enough
    if (best_correlation > cfg_.correlation_threshold) {
        result.valid = true;
        result.matched_position_ECEF = best_position;
        result.confidence = best_correlation;
        
        // Estimate uncertainty based on correlation
        // Higher correlation = lower uncertainty
        result.position_uncertainty_m = cfg_.grid_resolution_m * (2.0 - best_correlation);
        
        result.search_path = best_path;
        
        std::cout << "GRAVITY MAP MATCH FOUND!\n";
        std::cout << "  Correlation: " << best_correlation << "\n";
        std::cout << "  Position correction: " 
                  << (best_position - search_center).norm() << " m\n";
        std::cout << "  Estimated accuracy: " << result.position_uncertainty_m << " m\n";
    }
    
    return result;
}

void GravityMapMatcher::reset() {
    signature_buffer_.clear();
}

double GravityMapMatcher::computeCorrelation(const std::vector<double>& measured,
                                            const std::vector<double>& reference) const {
    if (measured.size() != reference.size() || measured.empty()) {
        return -1.0;
    }
    
    size_t n = measured.size();
    
    // Compute means
    double mean_m = std::accumulate(measured.begin(), measured.end(), 0.0) / n;
    double mean_r = std::accumulate(reference.begin(), reference.end(), 0.0) / n;
    
    // Compute correlation
    double num = 0.0, den_m = 0.0, den_r = 0.0;
    
    for (size_t i = 0; i < n; ++i) {
        double dm = measured[i] - mean_m;
        double dr = reference[i] - mean_r;
        num += dm * dr;
        den_m += dm * dm;
        den_r += dr * dr;
    }
    
    if (den_m == 0 || den_r == 0) return 0.0;
    
    return num / (std::sqrt(den_m) * std::sqrt(den_r));
}

std::vector<double> GravityMapMatcher::extractMapSignature(
    const GravityGradientProvider& gravity_model,
    const std::vector<Eigen::Vector3d>& path) const {
    
    std::vector<double> signature;
    
    for (const auto& pos : path) {
        // Get gravity at this position
        auto tensor = gravity_model.getGradient(pos);
        
        // Use trace as primary signature (could also use full tensor)
        double anomaly = tensor.T.trace() * 1e9;  // Convert to mGal-equivalent
        signature.push_back(anomaly);
    }
    
    return signature;
}

std::vector<Eigen::Vector3d> GravityMapMatcher::generateSearchGrid(
    const Eigen::Vector3d& center_ECEF) const {
    
    std::vector<Eigen::Vector3d> grid;
    
    // Convert to local tangent plane for grid generation
    // Simplified: use North-East-Down frame
    Eigen::Vector3d north = Eigen::Vector3d(0, 0, 1).cross(center_ECEF).normalized();
    Eigen::Vector3d east = center_ECEF.cross(north).normalized();
    
    // Generate grid
    int n_steps = cfg_.search_radius_m / cfg_.grid_resolution_m;
    
    for (int i = -n_steps; i <= n_steps; ++i) {
        for (int j = -n_steps; j <= n_steps; ++j) {
            double north_offset = i * cfg_.grid_resolution_m;
            double east_offset = j * cfg_.grid_resolution_m;
            
            // Skip points outside circular search radius
            if (std::sqrt(north_offset*north_offset + east_offset*east_offset) > cfg_.search_radius_m) {
                continue;
            }
            
            Eigen::Vector3d grid_point = center_ECEF + 
                north * north_offset + 
                east * east_offset;
            
            grid.push_back(grid_point);
        }
    }
    
    return grid;
}

double GravityMapMatcher::computeSSD(const std::vector<double>& measured,
                                    const std::vector<double>& reference) const {
    if (measured.size() != reference.size()) {
        return 1e9;  // Large error
    }
    
    double ssd = 0.0;
    for (size_t i = 0; i < measured.size(); ++i) {
        double diff = measured[i] - reference[i];
        ssd += diff * diff;
    }
    
    return ssd / measured.size();
}

std::vector<double> GravityMapMatcher::normalizeSignature(const std::vector<double>& sig) const {
    if (sig.empty()) return sig;
    
    // Compute mean and std dev
    double mean = std::accumulate(sig.begin(), sig.end(), 0.0) / sig.size();
    
    double var = 0.0;
    for (double val : sig) {
        var += (val - mean) * (val - mean);
    }
    var /= sig.size();
    double stddev = std::sqrt(var);
    
    // Normalize
    std::vector<double> normalized;
    if (stddev > 1e-10) {
        for (double val : sig) {
            normalized.push_back((val - mean) / stddev);
        }
    } else {
        // No variation, return zeros
        normalized.resize(sig.size(), 0.0);
    }
    
    return normalized;
}