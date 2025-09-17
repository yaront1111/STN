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
    
    // Extract measured signature using FULL tensor (10 components: anomaly + 9 tensor)
    std::vector<Eigen::VectorXd> measured_signature_multi;
    std::vector<Eigen::Vector3d> estimated_path;

    for (const auto& meas : signature_buffer_) {
        measured_signature_multi.push_back(meas.getFlattenedSignature());
        estimated_path.push_back(meas.position_ECEF);
    }
    
    // Normalize multi-dimensional signature for correlation
    auto measured_norm = normalizeMultiDimSignature(measured_signature_multi);
    
    // Get search center (current estimated position)
    Eigen::Vector3d search_center = signature_buffer_.back().position_ECEF;
    
    // Generate search grid
    auto search_grid = generateSearchGrid(search_center);
    
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
        
        // Extract multi-dimensional gravity signature from map at this path
        auto map_signature = extractMultiDimMapSignature(gravity_model, test_path);

        if (map_signature.empty()) continue;

        // Normalize and correlate multi-dimensional data
        auto map_norm = normalizeMultiDimSignature(map_signature);
        double correlation = computeMultiDimCorrelation(measured_norm, map_norm);
        
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
        
        // Progress report every 1000 points for large searches
        if (search_grid.size() > 1000 && points_evaluated % 1000 == 0) {
            std::cout << "  Evaluated " << points_evaluated << "/" << search_grid.size() 
                      << " points, best correlation: " << best_correlation << "\n";
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    // Check if match is good enough
    double correction_dist = (best_position - search_center).norm();

    // CRITICAL: Reject matches at search boundary - these are likely false positives!
    bool at_boundary = std::abs(correction_dist - cfg_.search_radius_m) < cfg_.grid_resolution_m;

    if (best_correlation > cfg_.correlation_threshold && !at_boundary) {
        result.valid = true;
        result.matched_position_ECEF = best_position;
        result.confidence = best_correlation;

        // Estimate uncertainty based on correlation
        // Higher correlation = lower uncertainty
        result.position_uncertainty_m = cfg_.grid_resolution_m * (2.0 - best_correlation);

        result.search_path = best_path;

        std::cout << "GRAVITY MAP MATCH FOUND!\n";
        std::cout << "  Correlation: " << best_correlation << "\n";
        std::cout << "  Position correction: " << correction_dist << " m\n";
        std::cout << "  Grid points evaluated: " << points_evaluated << "/" << search_grid.size() << "\n";
        std::cout << "  Using multi-dim signatures: " << measured_signature_multi[0].size() << " dimensions\n";
        std::cout << "  Estimated accuracy: " << result.position_uncertainty_m << " m\n";
    } else if (at_boundary) {
        std::cout << "MATCH REJECTED: Found at search boundary (likely false positive)\n";
        std::cout << "  Correlation was: " << best_correlation << "\n";
        std::cout << "  Distance: " << correction_dist << " m (boundary: " << cfg_.search_radius_m << " m)\n";
    }
    
    return result;
}

std::vector<GravityMapMatcher::MatchCandidate> GravityMapMatcher::findCandidates(
    const GravityGradientProvider& gravity_model,
    double min_correlation,
    int max_candidates) {

    std::vector<MatchCandidate> candidates;

    // Need minimum measurements
    if (signature_buffer_.size() < cfg_.min_measurements) {
        return candidates;
    }

    // Extract measured signature using FULL tensor
    std::vector<Eigen::VectorXd> measured_signature_multi;
    std::vector<Eigen::Vector3d> estimated_path;

    for (const auto& meas : signature_buffer_) {
        measured_signature_multi.push_back(meas.getFlattenedSignature());
        estimated_path.push_back(meas.position_ECEF);
    }

    // Normalize multi-dimensional signature
    auto measured_norm = normalizeMultiDimSignature(measured_signature_multi);

    // Get search center
    Eigen::Vector3d search_center = signature_buffer_.back().position_ECEF;

    // Generate search grid
    auto search_grid = generateSearchGrid(search_center);

    // Evaluate all grid points and collect candidates
    for (const auto& grid_point : search_grid) {
        // Compute offset
        Eigen::Vector3d offset = grid_point - search_center;

        // Apply offset to entire path
        std::vector<Eigen::Vector3d> test_path;
        for (const auto& pos : estimated_path) {
            test_path.push_back(pos + offset);
        }

        // Extract multi-dimensional gravity signature
        auto map_signature = extractMultiDimMapSignature(gravity_model, test_path);
        if (map_signature.empty()) continue;

        // Normalize and correlate
        auto map_norm = normalizeMultiDimSignature(map_signature);
        double correlation = computeMultiDimCorrelation(measured_norm, map_norm);

        // Check if this is a valid candidate
        if (correlation >= min_correlation) {
            MatchCandidate candidate;
            candidate.position_ECEF = grid_point;
            candidate.gravity_correlation = correlation;
            candidate.terrain_correlation = 0.0;  // Will be filled by terrain correlator
            candidate.combined_score = correlation;  // Initial score is gravity only
            candidate.path = test_path;  // This contains the full shifted path

            candidates.push_back(candidate);
        }
    }

    // Sort by gravity correlation (best first)
    std::sort(candidates.begin(), candidates.end(),
              [](const MatchCandidate& a, const MatchCandidate& b) {
                  return a.gravity_correlation > b.gravity_correlation;
              });

    // Limit to max_candidates
    if (candidates.size() > max_candidates) {
        candidates.resize(max_candidates);
    }

    std::cout << "Found " << candidates.size() << " gravity match candidates\n";
    for (size_t i = 0; i < std::min(size_t(5), candidates.size()); ++i) {
        std::cout << "  Candidate " << i+1 << ": correlation="
                  << candidates[i].gravity_correlation << "\n";
    }

    return candidates;
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

std::vector<Eigen::VectorXd> GravityMapMatcher::extractMultiDimMapSignature(
    const GravityGradientProvider& gravity_model,
    const std::vector<Eigen::Vector3d>& path) const {

    std::vector<Eigen::VectorXd> signature;

    for (const auto& pos : path) {
        // Get gravity gradient tensor at this position
        auto tensor_result = gravity_model.getGradient(pos);
        double anomaly = gravity_model.getAnomaly(pos);

        // Create 10D signature: 1 anomaly + 9 tensor components
        Eigen::VectorXd sig(10);
        sig(0) = anomaly;  // Already in mGal

        // Flatten the 3x3 tensor into 9 components
        int idx = 1;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                sig(idx++) = tensor_result.T(i, j);
            }
        }

        signature.push_back(sig);
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

std::vector<Eigen::VectorXd> GravityMapMatcher::normalizeMultiDimSignature(
    const std::vector<Eigen::VectorXd>& sig) const {

    if (sig.empty() || sig[0].size() == 0) return sig;

    int n_samples = sig.size();
    int n_dims = sig[0].size();

    // Initialize normalized vector with proper size
    std::vector<Eigen::VectorXd> normalized(n_samples);
    for (int i = 0; i < n_samples; ++i) {
        normalized[i] = Eigen::VectorXd(n_dims);
    }

    // Normalize each dimension independently
    for (int dim = 0; dim < n_dims; ++dim) {
        // Collect values for this dimension
        std::vector<double> dim_values;
        for (const auto& sample : sig) {
            dim_values.push_back(sample(dim));
        }

        // Compute mean and stddev for this dimension
        double mean = 0.0;
        for (double v : dim_values) mean += v;
        mean /= n_samples;

        double var = 0.0;
        for (double v : dim_values) {
            double diff = v - mean;
            var += diff * diff;
        }
        var /= n_samples;
        double stddev = std::sqrt(var);

        // Normalize and store
        for (int i = 0; i < n_samples; ++i) {
            if (stddev > 1e-10) {
                normalized[i](dim) = (sig[i](dim) - mean) / stddev;
            } else {
                normalized[i](dim) = 0.0;  // No variation in this dimension
            }
        }
    }

    return normalized;
}

double GravityMapMatcher::computeMultiDimCorrelation(
    const std::vector<Eigen::VectorXd>& measured,
    const std::vector<Eigen::VectorXd>& reference) const {

    if (measured.size() != reference.size() || measured.empty()) {
        return -1.0;
    }

    int n_samples = measured.size();
    int n_dims = measured[0].size();

    // Weight different components differently
    // Anomaly (dim 0) gets higher weight as it's more reliable
    // Tensor components (dims 1-9) get equal but lower weights
    std::vector<double> dim_weights(n_dims);
    dim_weights[0] = 2.0;  // Anomaly weight
    for (int i = 1; i < n_dims; ++i) {
        dim_weights[i] = 1.0;  // Tensor component weights
    }

    // Normalize weights
    double weight_sum = 0.0;
    for (double w : dim_weights) weight_sum += w;
    for (double& w : dim_weights) w /= weight_sum;

    // Compute weighted correlation across all dimensions
    double total_correlation = 0.0;

    for (int dim = 0; dim < n_dims; ++dim) {
        // Extract this dimension
        std::vector<double> meas_dim, ref_dim;
        for (int i = 0; i < n_samples; ++i) {
            meas_dim.push_back(measured[i](dim));
            ref_dim.push_back(reference[i](dim));
        }

        // Compute correlation for this dimension
        double dim_corr = computeCorrelation(meas_dim, ref_dim);

        // Weight and accumulate
        if (std::isfinite(dim_corr)) {
            total_correlation += dim_weights[dim] * dim_corr;
        }
    }

    return total_correlation;
}