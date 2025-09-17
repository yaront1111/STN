#include "hierarchical_map_matcher.h"
#include <algorithm>
#include <numeric>
#include <iostream>
#include <execution>
#include <vector>
#include <mutex>
#include <thread>

HierarchicalMapMatcher::HierarchicalMapMatcher(const Config& cfg) : cfg_(cfg) {
    signature_buffer_.clear();
}

void HierarchicalMapMatcher::addMeasurement(const GravityMapMatcher::GravityMeasurement& meas) {
    signature_buffer_.push_back(meas);

    // Keep buffer at maximum length
    while (signature_buffer_.size() > cfg_.signature_length) {
        signature_buffer_.pop_front();
    }
}

HierarchicalMapMatcher::HierarchicalResult HierarchicalMapMatcher::findMatch(
    const GravityGradientProvider& gravity_model) {

    HierarchicalResult result;
    auto start_time = std::chrono::high_resolution_clock::now();

    // Need minimum measurements
    if (signature_buffer_.size() < cfg_.min_measurements) {
        result.valid = false;
        return result;
    }

    // Extract measured signature
    std::vector<Eigen::VectorXd> measured_signature;
    std::vector<Eigen::Vector3d> estimated_path;

    for (const auto& meas : signature_buffer_) {
        measured_signature.push_back(meas.getFlattenedSignature());
        estimated_path.push_back(meas.position_ECEF);
    }

    // Normalize once for all levels
    auto measured_norm = normalizeSignature(measured_signature);

    // Start search from current estimated position
    Eigen::Vector3d search_center = signature_buffer_.back().position_ECEF;
    Eigen::Vector3d best_position = search_center;
    double best_correlation = -1.0;
    int total_evaluations = 0;

    // HIERARCHICAL SEARCH
    for (size_t level_idx = 0; level_idx < cfg_.levels.size(); ++level_idx) {
        const auto& level = cfg_.levels[level_idx];

        std::cout << "\n  Level " << (level_idx + 1) << " (" << level.name << "): ";
        std::cout << level.radius_m << "m radius, " << level.resolution_m << "m grid\n";

        // Search at this level
        int level_evals = 0;
        auto candidates = searchLevel(level, gravity_model, search_center, level_evals);
        total_evaluations += level_evals;

        if (candidates.empty()) {
            std::cout << "    No candidates found at level " << level.name << "\n";
            break;
        }

        // Sort by correlation
        std::sort(candidates.begin(), candidates.end(),
                 [](const auto& a, const auto& b) {
                     return a.gravity_correlation > b.gravity_correlation;
                 });

        // Report best at this level
        best_correlation = candidates[0].gravity_correlation;
        best_position = candidates[0].position_ECEF;
        result.level_correlations.push_back(best_correlation);

        std::cout << "    Best correlation: " << best_correlation;
        std::cout << " (evaluated " << level_evals << " points)\n";

        // Check for early termination
        if (cfg_.early_termination && best_correlation >= cfg_.excellent_correlation) {
            std::cout << "    EXCELLENT match found - stopping early!\n";
            break;
        }

        // Check if correlation meets threshold
        if (best_correlation < level.correlation_threshold) {
            std::cout << "    Correlation below threshold " << level.correlation_threshold << " - stopping\n";
            break;
        }

        // Update search center for next level
        search_center = best_position;

        // For final level, just use best position
        if (level_idx == cfg_.levels.size() - 1) {
            result.valid = true;
            result.matched_position_ECEF = best_position;
            result.final_correlation = best_correlation;

            // Estimate uncertainty based on resolution and correlation
            result.position_uncertainty_m = level.resolution_m * (2.0 - best_correlation);
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    result.search_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    result.total_evaluations = total_evaluations;

    if (result.valid) {
        double correction_dist = (result.matched_position_ECEF - estimated_path.back()).norm();
        std::cout << "\n  HIERARCHICAL MATCH FOUND!\n";
        std::cout << "  Final correlation: " << result.final_correlation << "\n";
        std::cout << "  Position correction: " << correction_dist << " m\n";
        std::cout << "  Total evaluations: " << total_evaluations << "\n";
        std::cout << "  Search time: " << result.search_time_ms.count() << " ms\n";
    }

    return result;
}

std::vector<GravityMapMatcher::MatchCandidate> HierarchicalMapMatcher::searchLevel(
    const SearchLevel& level,
    const GravityGradientProvider& gravity_model,
    const Eigen::Vector3d& search_center,
    int& evaluations) {

    std::vector<GravityMapMatcher::MatchCandidate> candidates;

    // Generate search grid for this level
    auto grid = generateLevelGrid(search_center, level.radius_m, level.resolution_m);

    // Get current estimated path
    std::vector<Eigen::Vector3d> estimated_path;
    for (const auto& meas : signature_buffer_) {
        estimated_path.push_back(meas.position_ECEF);
    }

    // Extract and normalize measured signature
    std::vector<Eigen::VectorXd> measured_signature;
    for (const auto& meas : signature_buffer_) {
        measured_signature.push_back(meas.getFlattenedSignature());
    }
    auto measured_norm = normalizeSignature(measured_signature);

    // Dynamic early termination threshold based on correlation statistics
    double dynamic_threshold = cfg_.excellent_correlation;
    const int SAMPLE_SIZE = std::min(50, static_cast<int>(grid.size() / 4));

    // Sample a subset to estimate correlation distribution
    if (grid.size() > SAMPLE_SIZE * 2) {
        std::vector<double> sample_correlations;
        for (int i = 0; i < SAMPLE_SIZE; i++) {
            int idx = (i * grid.size()) / SAMPLE_SIZE;  // Evenly distributed sampling
            const auto& test_point = grid[idx];

            Eigen::Vector3d offset = test_point - search_center;
            std::vector<Eigen::Vector3d> test_path;
            for (const auto& pos : estimated_path) {
                test_path.push_back(pos + offset);
            }

            auto map_sig = extractSignature(gravity_model, test_path);
            if (!map_sig.empty()) {
                auto map_norm = normalizeSignature(map_sig);
                double corr = computeCorrelation(measured_norm, map_norm);
                sample_correlations.push_back(corr);
            }
            evaluations++;
        }

        // Compute statistics
        if (!sample_correlations.empty()) {
            double mean = std::accumulate(sample_correlations.begin(),
                                         sample_correlations.end(), 0.0) / sample_correlations.size();
            double var = 0.0;
            for (double c : sample_correlations) {
                var += (c - mean) * (c - mean);
            }
            double stddev = std::sqrt(var / sample_correlations.size());

            // Set dynamic threshold at mean + 3*stddev
            dynamic_threshold = std::min(cfg_.excellent_correlation, mean + 3.0 * stddev);
            dynamic_threshold = std::max(dynamic_threshold, 0.85);  // At least 0.85
        }
    }

    // Evaluate remaining grid points with dynamic early termination
    for (const auto& grid_point : grid) {
        evaluations++;

        // Compute offset from estimated to grid point
        Eigen::Vector3d offset = grid_point - search_center;

        // Apply offset to entire path
        std::vector<Eigen::Vector3d> test_path;
        for (const auto& pos : estimated_path) {
            test_path.push_back(pos + offset);
        }

        // Extract gravity signature at this path
        auto map_signature = extractSignature(gravity_model, test_path);
        if (map_signature.empty()) continue;

        // Normalize and correlate
        auto map_norm = normalizeSignature(map_signature);
        double correlation = computeCorrelation(measured_norm, map_norm);

        // Add as candidate if above threshold
        if (correlation >= level.correlation_threshold) {
            GravityMapMatcher::MatchCandidate candidate;
            candidate.position_ECEF = grid_point;
            candidate.gravity_correlation = correlation;
            candidate.path = test_path;
            candidates.push_back(candidate);
        }

        // Dynamic early termination
        if (cfg_.early_termination && correlation >= dynamic_threshold) {
            std::cout << "    Early termination at correlation " << correlation
                     << " (threshold=" << dynamic_threshold << ")\n";
            break;
        }
    }

    // Keep only top N candidates
    if (candidates.size() > level.max_candidates) {
        std::partial_sort(candidates.begin(),
                         candidates.begin() + level.max_candidates,
                         candidates.end(),
                         [](const auto& a, const auto& b) {
                             return a.gravity_correlation > b.gravity_correlation;
                         });
        candidates.resize(level.max_candidates);
    }

    return candidates;
}

std::vector<Eigen::Vector3d> HierarchicalMapMatcher::generateLevelGrid(
    const Eigen::Vector3d& center,
    double radius_m,
    double resolution_m) {

    std::vector<Eigen::Vector3d> grid;

    // Convert to local tangent plane
    Eigen::Vector3d north = Eigen::Vector3d(0, 0, 1).cross(center).normalized();
    Eigen::Vector3d east = center.cross(north).normalized();

    // Calculate grid steps
    int n_steps = static_cast<int>(radius_m / resolution_m);

    // Generate grid points
    for (int i = -n_steps; i <= n_steps; ++i) {
        for (int j = -n_steps; j <= n_steps; ++j) {
            double north_offset = i * resolution_m;
            double east_offset = j * resolution_m;

            // Skip points outside circular radius
            double dist = std::sqrt(north_offset*north_offset + east_offset*east_offset);
            if (dist > radius_m) continue;

            Eigen::Vector3d grid_point = center +
                north * north_offset +
                east * east_offset;

            grid.push_back(grid_point);
        }
    }

    return grid;
}

std::vector<Eigen::VectorXd> HierarchicalMapMatcher::extractSignature(
    const GravityGradientProvider& gravity_model,
    const std::vector<Eigen::Vector3d>& path) const {

    std::vector<Eigen::VectorXd> signature;
    signature.reserve(path.size());  // Pre-allocate for performance

    for (const auto& pos : path) {
        // Get gravity gradient (anomaly is computed from gradient, no need for separate call)
        auto tensor_result = gravity_model.getGradient(pos);

        // Create 10D signature: 1 anomaly + 9 tensor components
        Eigen::VectorXd sig(10);

        // Anomaly is trace of gradient tensor * 0.1 (convert Eötvös to mGal)
        sig(0) = tensor_result.T.trace() * 0.1;

        // Flatten the 3x3 tensor - use more efficient single-loop approach
        Eigen::Map<Eigen::VectorXd>(sig.data() + 1, 9) =
            Eigen::Map<const Eigen::VectorXd>(tensor_result.T.data(), 9);

        signature.push_back(sig);
    }

    return signature;
}

std::vector<Eigen::VectorXd> HierarchicalMapMatcher::normalizeSignature(
    const std::vector<Eigen::VectorXd>& sig) const {

    if (sig.empty() || sig[0].size() == 0) return sig;

    int n_samples = sig.size();
    int n_dims = sig[0].size();

    std::vector<Eigen::VectorXd> normalized(n_samples);
    for (int i = 0; i < n_samples; ++i) {
        normalized[i] = Eigen::VectorXd(n_dims);
    }

    // Normalize each dimension independently
    for (int dim = 0; dim < n_dims; ++dim) {
        // Compute mean and stddev
        double mean = 0.0;
        for (const auto& sample : sig) {
            mean += sample(dim);
        }
        mean /= n_samples;

        double var = 0.0;
        for (const auto& sample : sig) {
            double diff = sample(dim) - mean;
            var += diff * diff;
        }
        var /= n_samples;
        double stddev = std::sqrt(var);

        // Normalize
        for (int i = 0; i < n_samples; ++i) {
            if (stddev > 1e-10) {
                normalized[i](dim) = (sig[i](dim) - mean) / stddev;
            } else {
                normalized[i](dim) = 0.0;
            }
        }
    }

    return normalized;
}

double HierarchicalMapMatcher::computeCorrelation(
    const std::vector<Eigen::VectorXd>& measured,
    const std::vector<Eigen::VectorXd>& reference) const {

    if (measured.size() != reference.size() || measured.empty()) {
        return -1.0;
    }

    int n_samples = measured.size();
    int n_dims = measured[0].size();

    // Weight different components
    std::vector<double> dim_weights(n_dims);
    dim_weights[0] = 2.0;  // Anomaly weight
    for (int i = 1; i < n_dims; ++i) {
        dim_weights[i] = 1.0;  // Tensor weights
    }

    // Normalize weights
    double weight_sum = std::accumulate(dim_weights.begin(), dim_weights.end(), 0.0);
    for (double& w : dim_weights) w /= weight_sum;

    // Compute weighted correlation using vectorized operations
    double total_correlation = 0.0;

    // Process all dimensions at once for better cache locality
    for (int dim = 0; dim < n_dims; ++dim) {
        // Extract dimension using pre-allocated vectors
        std::vector<double> meas_dim(n_samples), ref_dim(n_samples);
        for (int i = 0; i < n_samples; ++i) {
            meas_dim[i] = measured[i](dim);
            ref_dim[i] = reference[i](dim);
        }

        // Compute correlation for this dimension
        double sum_xy = 0, sum_x = 0, sum_y = 0, sum_x2 = 0, sum_y2 = 0;
        for (size_t i = 0; i < n_samples; ++i) {
            sum_xy += meas_dim[i] * ref_dim[i];
            sum_x += meas_dim[i];
            sum_y += ref_dim[i];
            sum_x2 += meas_dim[i] * meas_dim[i];
            sum_y2 += ref_dim[i] * ref_dim[i];
        }

        double numerator = n_samples * sum_xy - sum_x * sum_y;
        double denominator = std::sqrt((n_samples * sum_x2 - sum_x * sum_x) *
                                      (n_samples * sum_y2 - sum_y * sum_y));

        double dim_corr = 0.0;
        if (denominator > 1e-10) {
            dim_corr = numerator / denominator;
        }

        if (std::isfinite(dim_corr)) {
            total_correlation += dim_weights[dim] * dim_corr;
        }
    }

    return total_correlation;
}

void HierarchicalMapMatcher::reset() {
    signature_buffer_.clear();
}